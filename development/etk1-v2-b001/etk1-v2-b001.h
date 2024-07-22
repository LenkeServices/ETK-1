#define IN1 33
#define IN2 32
#define IN3 35
#define IN4 34

#define OUT1 25  
#define OUT2 26   
#define OUT3 2
#define OUT4 27
#define OUT5 14
#define OUT6 13

#define BOARD_LED 12
/*
ESP32 32BITS
INTEGER   -2,147,483,648 to 2,147,483,647

ATMEGA 328P
INTEGER   -32,768 to 32,767
*/

String hversion = "1.2";
String version = "ETK1-V2-B001";

int SERIAL_PCK_SIZE = 20;

bool hasRtc = true;
bool clientConnect = false;

int weights[] = { 0, 0, 0, 0, 0};
bool clientConnected = false;
String wText = "";
String pck_start = "";
int pckLimit = 0;
String data = "";
String macaddress = "";
String pckSctCmd = "";
String pckSct = "";
String pcode = "";
String tagrfid = "0";

#define RDM6300_RX_PIN 15
uint8_t Payload[6];


struct s_etk {
  bool free_apt = false;
  int id = 1;                     // p0
  bool pwr_rfid = false;          // p1
  int w_min = 200;                // p2
  int w_max = 6000;               // p3
  int w_capacity = 6000;          // p4
  int w_variation = 30000;        // p5
  bool apt_mode = true;           // p6 0:estabilizacao 1:env
  String productcode = "0";       // p7
  short portSerial = 0;           // p8 0:serial0 1:serial1
  int portBaud = 9600;            // p9
  short qtd_w_apt = 1;            // p10
  String rfidusr = "0";
  String rfidprd = "0";
  int weight = 0;
  int w_stable = 0;
  char c_stable = 0x00;
  char c_signal = 0x00;
  int w1 = 0;
  int w2 = 0;
};
s_etk etk;


struct s_lan {
  bool connected = false;
  bool wifi = true;               // p50
  bool dhcp = true;               // p51
  String ip = "192.168.4.2";      // p52
  String msk = "255.255.255.0";   // p53 
  String gtw = "192.168.4.1";     // p54
  String dns = "192.168.4.1";     // p55 
  String ssid = "lenkelog";       // p56
  String pwd = "38844266";        // p57
  String tcp_hip = "192.168.4.1"; // p58
  unsigned int tcp_hport = 13085; // p59
  bool use_tcp = false;            // p60 1:TCP CLIENT 0:NOT
};
s_lan lan;


String mac_str = "";
uint8_t mac[6];
IPAddress local_ip(192, 168, 4, 2);
IPAddress gateway(192, 168, 4, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress dns(192, 168, 4, 1);

