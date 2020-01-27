// Xilinx virtual JTAG cable with ESP update, logging and WIFI Manager

#include <ESP8266WiFi.h>          //https://github.com/esp8266/Arduino

//needed for library
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager
#include <ESP8266mDNS.h>
#include <ESP8266HTTPUpdateServer.h>
#include <WiFiUdp.h>

const char* host = "Xilinx-JTAG-Server";


#define GPIO_TDI    D6
#define GPIO_TDO    D4
#define GPIO_TCK    D7
#define GPIO_TMS    D5

//#define VERBOSE
#define MAX_WRITE_SIZE  512

IPAddress syslogServer(192, 168, 0, 62);

const int port = 2542;

WiFiServer server(port);
WiFiClient client;
WiFiUDP udp;

ESP8266WebServer httpServer(80);
ESP8266HTTPUpdateServer httpUpdater;

int jtag_state;

// JTAG buffers
uint8_t cmd[16];
uint8_t buffer[1024], result[512];

enum {

  test_logic_reset, run_test_idle,

  select_dr_scan, capture_dr, shift_dr,
  exit1_dr, pause_dr, exit2_dr, update_dr,

  select_ir_scan, capture_ir, shift_ir,
  exit1_ir, pause_ir, exit2_ir, update_ir,

  num_states
};

void sendUdpSyslog(String msgtosend)
{
  unsigned int msg_length = msgtosend.length();
  byte* p = (byte*)malloc(msg_length);
  memcpy(p, (char*) msgtosend.c_str(), msg_length);
  udp.beginPacket(syslogServer, 514);
  udp.write(host);
  udp.write(' ');
  udp.write(p, msg_length);
  udp.endPacket();
  free(p);
}

int jtag_step(int state, int tms) {

  const int next_state[num_states][2] = {

    [test_logic_reset] = {run_test_idle, test_logic_reset},
    [run_test_idle] = {run_test_idle, select_dr_scan},

    [select_dr_scan] = {capture_dr, select_ir_scan},
    [capture_dr] = {shift_dr, exit1_dr},
    [shift_dr] = {shift_dr, exit1_dr},
    [exit1_dr] = {pause_dr, update_dr},
    [pause_dr] = {pause_dr, exit2_dr},
    [exit2_dr] = {shift_dr, update_dr},
    [update_dr] = {run_test_idle, select_dr_scan},

    [select_ir_scan] = {capture_ir, test_logic_reset},
    [capture_ir] = {shift_ir, exit1_ir},
    [shift_ir] = {shift_ir, exit1_ir},
    [exit1_ir] = {pause_ir, update_ir},
    [pause_ir] = {pause_ir, exit2_ir},
    [exit2_ir] = {shift_ir, update_ir},
    [update_ir] = {run_test_idle, select_dr_scan}
  };

  return next_state[state][tms];
}

int sread(void *target, int len) {

  uint8_t *t = (uint8_t *) target;

  while (len) {

    int r = client.read(t, len);
    if (r <= 0)
      return r;
    t += r;
    len -= r;
  }

  return 1;
}

//for LED status
#include <Ticker.h>
Ticker ticker;

void tick()
{
  //toggle state
  int state = digitalRead(BUILTIN_LED);  // get the current state of GPIO1 pin
  digitalWrite(BUILTIN_LED, !state);     // set pin to the opposite state
}

//gets called when WiFiManager enters configuration mode
void configModeCallback (WiFiManager *myWiFiManager) {
  Serial.println("Entered config mode");
  Serial.println(WiFi.softAPIP());
  //if you used auto generated SSID, print it
  Serial.println(myWiFiManager->getConfigPortalSSID());
  //entered config mode, make led toggle faster
  ticker.attach(0.2, tick);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  
  //set led pin as output
  pinMode(BUILTIN_LED, OUTPUT);
  // start ticker with 0.5 because we start in AP mode and try to connect
  ticker.attach(0.6, tick);

  //WiFiManager
  //Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;
  //reset settings - for testing
  //wifiManager.resetSettings();

  //set callback that gets called when connecting to previous WiFi fails, and enters Access Point mode
  wifiManager.setAPCallback(configModeCallback);

  //fetches ssid and pass and tries to connect
  //if it does not connect it starts an access point with the specified name
  //here  "AutoConnectAP"
  //and goes into a blocking loop awaiting configuration
  if (!wifiManager.autoConnect()) {
    Serial.println("failed to connect and hit timeout");
    //reset and try again, or maybe put it to deep sleep
    ESP.reset();
    delay(1000);
  }

  //if you get here you have connected to the WiFi
  Serial.println("connected...yeey :)");
  ticker.detach();
  //keep LED on
  digitalWrite(BUILTIN_LED, LOW);
  sendUdpSyslog("Started");
  MDNS.begin(host);

  httpUpdater.setup(&httpServer);
  httpServer.begin();

  MDNS.addService("http", "tcp", 80);
  Serial.printf("HTTPUpdateServer ready! Open http://%s.local/update in your browser\n", host);


  Serial.print("Starting XVC Server on port ");
  Serial.println(port);
  sendUdpSyslog("Starting XVC Server");

  pinMode(GPIO_TDI, OUTPUT);
  pinMode(GPIO_TDO, INPUT);
  pinMode(GPIO_TCK, OUTPUT);
  pinMode(GPIO_TMS, OUTPUT);

  server.begin();
  server.setNoDelay(true);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  httpServer.handleClient();
  MDNS.update();

  start: if (!client.connected()) {
    
    // try to connect to a new client
    client = server.available();
    
  } else {
    
    // read data from the connected client
    if (client.available()) {

      while (client.connected()) {
      
        int i;
        int seen_tlr = 0;
      
        do {
      
          if (sread(cmd, 6) != 1)
            goto start;
      
          if (memcmp(cmd, "shift:", 6)) {

            cmd[6] = 0;
            Serial.print("invalid cmd ");
            Serial.println((char *)cmd);
            goto start;
          }

          int len;
          if (sread(&len, 4) != 1) {

            Serial.println("reading length failed\n");
            goto start;
          }
      
          int nr_bytes = (len + 7) / 8;

#ifdef VERBOSE

            Serial.print("len = ");
            Serial.print(len);
            Serial.print(" nr_bytes = ");
            Serial.println(nr_bytes);
#endif
      
          if (nr_bytes * 2 > sizeof(buffer)) {

            Serial.println("buffer size exceeded");
            goto start;
          }
      
          if (sread(buffer, nr_bytes * 2) != 1) {

            Serial.println("reading data failed\n");
            goto start;
          }
             
          memset((uint8_t *)result, 0, nr_bytes);
      
#ifdef VERBOSE

            Serial.print("#len=");
            Serial.print(nr_bytes);
            Serial.print("#");
      
            for (i = 0; i < nr_bytes * 2; ++i)
              Serial.print(buffer[i], HEX);
      
            Serial.println();
#endif
      
          // Only allow exiting if the state is rti and the IR
          // has the default value (IDCODE) by going through test_logic_reset.
          // As soon as going through capture_dr or capture_ir no exit is
          // allowed as this will change DR/IR.
      
          seen_tlr = (seen_tlr || jtag_state == test_logic_reset) && (jtag_state != capture_dr) && (jtag_state != capture_ir);
      
          //
          // Due to a weird bug(??) xilinx impacts goes through another "capture_ir"/"capture_dr" cycle after
          // reading IR/DR which unfortunately sets IR to the read-out IR value.
          // Just ignore these transactions.
      
          if ((jtag_state == exit1_ir && len == 5 && buffer[0] == 0x17) || (jtag_state == exit1_dr && len == 4 && buffer[0] == 0x0b)) {

#ifdef VERBOSE
              Serial.print("ignoring bogus jtag state movement in jtag_state ");
              Serial.println(jtag_state);
#endif
          } else
            for (i = 0; i < len; ++i) {
              // Do the actual cycle.
              int tms = !!(buffer[i / 8] & (1 << (i & 7)));
              int tdi = !!(buffer[nr_bytes + i / 8] & (1 << (i & 7)));
              result[i / 8] |= digitalRead(GPIO_TDO) << (i & 7);
              digitalWrite(GPIO_TMS, tms);
              digitalWrite(GPIO_TDI, tdi);
              digitalWrite(GPIO_TCK, 1);
              digitalWrite(GPIO_TCK, 0);
              
              // Track the state.
              jtag_state = jtag_step(jtag_state, tms);
            }
  
          if(client) {
            
            if(client.write((const uint8_t *)result, nr_bytes) != nr_bytes) {

              Serial.print("write error - nbytes =");
              Serial.println(nr_bytes);
              goto start;
            }
            
          } else {
  
            Serial.println("client error !!");
          }

#ifdef VERBOSE
            Serial.print("jtag state ");
            Serial.println(jtag_state);
#endif
      
        } while (!(seen_tlr && jtag_state == run_test_idle));
      }
    }
  }  
}
