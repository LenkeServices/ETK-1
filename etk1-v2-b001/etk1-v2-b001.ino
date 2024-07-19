#define ETK1V2

/*
COMPILACAO
Flash Frequency:    80Mhz
Flash Mode:         DIO OU QIO depende do chip
Flash Size:         4Mb
*/

#include <WiFi.h>
#include <Preferences.h>
#include <WiFiClient.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <ETH.h>
#include <SPI.h>
#include <RTClib.h>
#include <SPIFFS.h>
#include <rdm6300.h>
#include "esp_mac.h"

// --- ETHERNET ---
// Set this to 1 to enable dual Ethernet support
#define USE_TWO_ETH_PORTS 0
#ifndef ETH_PHY_TYPE
#define ETH_PHY_TYPE ETH_PHY_W5500
#define ETH_PHY_ADDR 1
#define ETH_PHY_CS 5
#define ETH_PHY_IRQ 4
#define ETH_PHY_RST 0
#endif
// SPI pins
#define ETH_SPI_SCK 18
#define ETH_SPI_MISO 19
#define ETH_SPI_MOSI 23
// ----------------

#include "etk1-v2-b001.h"

// PARTITION SCHEME - MINIMAL SPIFFS
//

TaskHandle_t Task1;
HardwareSerial serial1(2);


WiFiUDP udp;
RTC_DS1307 rtc;
WiFiClient client;
Preferences preferences;
Rdm6300 rdm6300;


void sendUdp(String str) {
  Serial.println("UDP > " + str);
  udp.beginPacket(udp.remoteIP(), udp.remotePort());
  udp.println(str);
  udp.endPacket();
}


void onEvent(arduino_event_id_t event, arduino_event_info_t info) {
  switch (event) {
    case ARDUINO_EVENT_ETH_START:
      Serial.println("ETH Started");
      //set eth hostname here
      ETH.setHostname("esp32-eth0");
      break;
    case ARDUINO_EVENT_ETH_CONNECTED: Serial.println("ETH Connected"); break;
    case ARDUINO_EVENT_ETH_GOT_IP:
      Serial.printf("ETH Got IP: '%s'\n", esp_netif_get_desc(info.got_ip.esp_netif));
      Serial.println(ETH);
      lan.connected = true;
      break;
    case ARDUINO_EVENT_ETH_LOST_IP:
      Serial.println("ETH Lost IP");
      lan.connected = false;
      break;
    case ARDUINO_EVENT_ETH_DISCONNECTED:
      Serial.println("ETH Disconnected");
      lan.connected = false;
      break;
    case ARDUINO_EVENT_ETH_STOP:
      Serial.println("ETH Stopped");
      lan.connected = false;
      break;
    default: break;
  }
}


void f_iniWifiStandard() {
  Serial.println("CONECTANDO lenkelog");
  WiFi.begin("lenkelog", "38844266");
  int timeout = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    if (timeout > 8) {
      break;
    }
    timeout++;
  }
  Serial.print("Conectado a rede wi-fi ");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
  if (WiFi.status() == WL_CONNECTED) {
    lan.connected = true;
  }
}


void setup() {
  // carrega preferencias
  f_preferences();
  //
  Serial.begin(etk.portBaud);
  serial1.begin(etk.portBaud, SERIAL_8N1, 17, 16);
  delay(300);
  Serial.println("");
  Serial.println("Version: " + version);
  //
  delay(100);
  //
  pinMode(IN1, INPUT_PULLUP);
  pinMode(IN2, INPUT_PULLUP);
  pinMode(IN3, INPUT_PULLUP);
  pinMode(IN4, INPUT_PULLUP);
  //
  pinMode(OUT1, OUTPUT);
  pinMode(OUT2, OUTPUT);
  pinMode(OUT3, OUTPUT);
  pinMode(OUT4, OUTPUT);
  pinMode(OUT5, OUTPUT);
  pinMode(OUT6, OUTPUT);
  pinMode(BOARD_LED, OUTPUT);
  //
  digitalWrite(OUT1, LOW);
  digitalWrite(OUT2, LOW);
  digitalWrite(OUT3, LOW);
  digitalWrite(OUT4, LOW);
  digitalWrite(OUT5, LOW);
  digitalWrite(OUT6, LOW);
  digitalWrite(BOARD_LED, HIGH);
  //
  rdm6300.begin(RDM6300_RX_PIN);
  //
  f_readMacAddress();
  f_iniWifiStandard();
  f_iniSpiffs();
  //
  // se nao se conectao na wifi de config entao busca padrao
  if (!lan.connected) {
    // se e uma configuracao wifi
    if (lan.wifi) {
      // se e uma conexao dhcp
      if (lan.dhcp) {
        WiFi.begin(lan.ssid.c_str(), lan.pwd.c_str());
        int timeout = 0;
        while (WiFi.status() != WL_CONNECTED) {
          delay(500);
          Serial.print(".");
          if (timeout > 10) {
            break;
          }
          timeout++;
        }
      } else {
        // senao e uma configuracao ip fixo
        local_ip.fromString(lan.ip.c_str());
        gateway.fromString(lan.gtw.c_str());
        subnet.fromString(lan.msk.c_str());
        dns.fromString(lan.dns.c_str());
        WiFi.mode(WIFI_STA);
        if (!WiFi.config(local_ip, gateway, subnet, dns)) {
          Serial.println("STA falhou para configuração");
        }
        WiFi.begin(lan.ssid.c_str(), lan.pwd.c_str());
        int timeout = 0;
        while (WiFi.status() != WL_CONNECTED) {
          delay(500);
          Serial.print(".");
          if (timeout > 10) {
            break;
          }
          timeout++;
        }
      }
      Serial.print("IP : ");
      Serial.println(WiFi.localIP());
      Serial.print("MSK: ");
      Serial.println(WiFi.subnetMask());
      Serial.print("GTW : ");
      Serial.println(WiFi.gatewayIP());
      Serial.print("DNS : ");
      Serial.println(WiFi.dnsIP());
    } else {
      // senao e uma configuracao etherenet
      Network.onEvent(onEvent);
      SPI.begin(ETH_SPI_SCK, ETH_SPI_MISO, ETH_SPI_MOSI);
      ETH.begin(ETH_PHY_TYPE, ETH_PHY_ADDR, ETH_PHY_CS, ETH_PHY_IRQ, ETH_PHY_RST, SPI);
      // se e uma conexao dhcp
      if (lan.dhcp) {
        ETH.config(mac);
      } else {
        // se nao for dhcp entao pega ip fixo
        ETH.config(
          local_ip,
          gateway,
          subnet,
          dns);
      }
      Serial.print("IP : ");
      Serial.println(ETH.localIP());
      Serial.print("MSK: ");
      Serial.println(ETH.subnetMask());
      Serial.print("GTW : ");
      Serial.println(ETH.gatewayIP());
      Serial.print("DNS : ");
      Serial.println(ETH.dnsIP());
    }
  }
  // verifica se encontra o rtc
  if (!rtc.begin()) {
    Serial.println("RTC NAO ENCONTRADO");
    Serial.flush();
    hasRtc = false;
  }
  //
  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else  // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
      sendUdp("U; firmware atualizado");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      sendUdp("U;" + String((progress / (total / 100))));
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      sendUdp("U; erro " + error);
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });
  //
  ArduinoOTA.begin();
  udp.begin(49000);
  //
  xTaskCreatePinnedToCore(
    task1Func, 
    "Task1", 
    10000,   
    NULL,    
    10,       
    &Task1,  
    0);     
  //
  delay(500);
  Serial.println(version);
  Serial.println("SISTEMA INICIADO");
  //
  if (etk.pwr_rfid) {
    digitalWrite(OUT1, LOW);
    digitalWrite(OUT2, LOW);
  } else {
    digitalWrite(OUT1, LOW);
    digitalWrite(OUT2, HIGH);
  }
  //
  digitalWrite(BOARD_LED, LOW);
  //
}


void loop() {
  serial_loop();
  udp_loop();
  rfid_loop();
  delay(1);
}

void task1Func(void *pvParameters) {
  for (;;) {
    if (lan.use_tcp) {
      if (!clientConnect) {
        if (client.connect(lan.tcp_hip.c_str(), lan.tcp_hport)) {
          clientConnect = true;
        } else {
          Serial.println("BUSCANDO TCP");
          delay(1000);
          clientConnect = false;
        }
      } else {
        if (client.connected()) {
          if (client.available()) {
            char c = client.read();
            if (c == 0x02) {
              pckSct = "";
              pckSctCmd = "";
            } else if (c == 0x0d) {
              Serial.print("COMANDO: ");
              Serial.print(pckSct);
              Serial.print("VALUE: ");
              pckSctCmd = "";
              pckSctCmd.concat(pckSct[0]);
              pckSctCmd.concat(pckSct[1]);
              pckSct = pckSct.substring(2);
              Serial.println(pckSctCmd);
              tcp_loop(pckSctCmd, pckSct);
              pckSct = "";
              pckSctCmd = "";
            } else {
              pckSct.concat(c);
            }
            //Serial.write(c);
          }
        } else {
          clientConnect = false;
          client.stop();
        }
      }
    }
    // ouvinte do atualizador online
    ArduinoOTA.handle();
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}


void serial_proccess() {
  pckLimit = 0;
  etk.weight = wText.substring(0, 8).toInt();
  etk.c_stable = wText.substring(8, 9)[0];
  etk.c_signal = wText.substring(9, 10)[0];
  if (etk.c_signal == 'N') {
    etk.weight = etk.weight * -1;
  }
  wText = "";
  if (etk.weight < etk.w_min || etk.apt_mode) {
    etk.free_apt = true;
  }

  if (etk.free_apt) {
    if (etk.weight > etk.w_min) {
      if (!etk.apt_mode) {
        addQueue();
      } else {
        f_registerWeight();
      }
    }
  }
}


void serial_loop() {
  if (Serial.available()) {
    if (pckLimit > SERIAL_PCK_SIZE) {
      pckLimit = 0;
      wText = "";
    }
    char c = char(serial1.read());
    if (c == 0x0D) {
      serial_proccess();
      pckLimit = 0;
    } else {
      pckLimit++;
      wText.concat(c);
    }
  }
  //
  if (serial1.available()) {
    if (pckLimit > SERIAL_PCK_SIZE) {
      pckLimit = 0;
      wText = "";
    }
    char c = char(serial1.read());
    if (c == 0x0D) {
      serial_proccess();
      pckLimit = 0;
    } else {
      pckLimit++;
      wText.concat(c);
    }
  }
}


void rfid_loop() {
  if (rdm6300.get_new_tag_id()) {
    tagrfid = rdm6300.get_tag_id();
    Serial.println(tagrfid);
    if (etk.pwr_rfid) {
      if (etk.rfidusr == "0")
      // se liga balanca por rfid
      {
        etk.rfidusr = tagrfid;
        digitalWrite(OUT2, HIGH);
        // liga a balanca
        delay(500);
        Serial.println("LIGANDO BALANCA");
        digitalWrite(OUT1, HIGH);
        delay(2000);
        digitalWrite(OUT1, LOW);
      } else if (etk.rfidusr == tagrfid) {
        Serial.println("DESLIGANDO BALANCA...");
        digitalWrite(OUT2, LOW);
        etk.rfidusr = "0";
        etk.rfidprd = "0";
      } else {
        etk.rfidprd = tagrfid;
        f_serial_print("@SC3");  // PRD
        delay(250);
        f_serial_print("@SC0");  // ON
      }
    } else
    // se nao liga balanca por rfid
    {
      if (etk.rfidusr == "0") {
        etk.rfidusr = tagrfid;
        f_serial_print("@SC2");  // USER
        delay(250);
        f_serial_print("@SC0");  // ON
      } else if (etk.rfidusr == tagrfid) {
        etk.rfidusr = "0";
        etk.rfidprd = "0";
        f_serial_print("@SC2");  // USER
        delay(250);
        f_serial_print("@SC1");  // OFF
      } else {
        etk.rfidprd = tagrfid;
        f_serial_print("@SC3");  // PRD
        delay(250);
        f_serial_print("@SC0");  // ON
      }
    }
    sendUdp("1;" + etk.rfidusr + ";" + etk.rfidprd);
  }
}

void f_serial_print(String txt) {
  if (etk.portSerial == 0) {
    Serial.println(txt);
  } else {
    serial1.println(txt);
  }
}

void f_registerWeight() {
  if (lan.use_tcp) {
    sendTcpWeight();
    sendUdp("L;" + String(etk.weight));
  } else {
    sendUdp("L;" + String(etk.weight));
  }
  etk.free_apt = false;
}


void sendTcpWeight() {
  String pck = etk.productcode;
  pck.concat(";");
  pck.concat(etk.weight);
  client.println(pck);
}


void addQueue() {
  weights[4] = weights[3];
  weights[3] = weights[2];
  weights[2] = weights[1];
  weights[1] = weights[0];
  weights[0] = etk.weight;
  double avg = (weights[4] + weights[3] + weights[2] + weights[1] + weights[0]) / 5;
  //Serial.println(String(weights[0]) + " - " + String(weights[1]) + " - " + String(weights[2]) + " - " + String(weights[3]) + " - " + String(weights[4]));
  if (etk.weight >= etk.w_min && etk.weight <= etk.w_max) {
    Serial.println("PESO ESTAVEL");
    if (client.connected()) {
      if (etk.weight > (avg - etk.w_variation) && etk.weight < (avg + etk.w_variation)) {
        f_registerWeight();
        f_serial_print("@SC4");
        weights[4] = 0;
        weights[3] = 0;
        weights[2] = 0;
        weights[1] = 0;
        weights[0] = 0;
      }
    }
  }
}


// VERIFICAR
void tcp_loop(String p_cmd, String p_value) {
  /*
  CODIGOS TCP 
  00 get params
  21 set variation
  23 set weight min
  24 set weight max
  25 set tcp_hip 
  26 set tcp_hport 
  27 set productcode 
  51 set ip
  52 set gateway
  53 set dns
  54 set mask
  55 set ssid
  56 set password
  99 restart
  */
  if (p_cmd == "00") {
    String s = "";
    s.concat("0;");
    s.concat(lan.ip);
    s.concat(";");
    s.concat(lan.msk);
    s.concat(";");
    s.concat(lan.dns);
    s.concat(";");
    s.concat(lan.gtw);
    s.concat(";");
    s.concat(lan.ssid);
    s.concat(";");
    s.concat(lan.pwd);
    s.concat(";");
    s.concat(etk.w_variation);
    s.concat(";");
    s.concat(etk.w_min);
    s.concat(";");
    s.concat(etk.w_min);
    s.concat(";");
    s.concat(etk.w_max);
    s.concat(";");
    s.concat(lan.tcp_hip);
    s.concat(";");
    s.concat(lan.tcp_hport);
    s.concat(";");
    s.concat(etk.productcode);
    Serial.print(s);
    client.println(s);
  } else if (p_cmd == "21") {
    etk.w_variation = p_value.toInt();
    preferences.putInt("p5", etk.w_variation);
    Serial.print("w_variation: ");
    Serial.println(etk.w_variation);
    /*} else if (p_cmd == "22") {
    minweight = p_value.toInt();
    preferences.putInt("minweight", minweight);
    Serial.println("minweight " + minweight);*/
  } else if (p_cmd == "23") {
    etk.w_min = p_value.toInt();
    preferences.putInt("p2", etk.w_min);
    Serial.print("w_min: ");
    Serial.println(etk.w_min);
  } else if (p_cmd == "24") {
    etk.w_max = p_value.toInt();
    preferences.putInt("p3", etk.w_max);
    Serial.print("w_max: ");
    Serial.println(etk.w_max);
  } else if (p_cmd == "25") {
    lan.tcp_hip = p_value;
    preferences.putString("p58", lan.tcp_hip);
    Serial.println("HIp: ");
    Serial.println(lan.tcp_hip);
  } else if (p_cmd == "26") {
    lan.tcp_hport = p_value.toInt();
    preferences.putInt("p59", lan.tcp_hport);
    Serial.println("HPort: ");
    Serial.println(lan.tcp_hport);
  } else if (p_cmd == "27") {
    etk.productcode = p_value;
    Serial.println("pcode: " + etk.productcode);
    preferences.putString("p7", etk.productcode);
  } else if (p_cmd == "51") {
    lan.ip = p_value;
    preferences.putString("p52", lan.ip.c_str());
    Serial.print("ip: ");
    Serial.println(lan.ip);
  } else if (p_cmd == "52") {
    lan.gtw = p_value;
    preferences.putString("p54", lan.gtw.c_str());
    Serial.print("gtw: ");
    Serial.println(lan.gtw);
  } else if (p_cmd == "53") {
    lan.dns = p_value;
    preferences.putString("p55", lan.dns.c_str());
    Serial.print("dns: ");
    Serial.println(lan.dns);
  } else if (p_cmd == "54") {
    lan.msk = p_value;
    preferences.putString("p53", lan.msk.c_str());
    Serial.print("msk: ");
    Serial.println(lan.msk);
  } else if (p_cmd == "55") {
    lan.ssid = p_value;
    preferences.putString("p56", lan.ssid.c_str());
    Serial.print("ssid: ");
    Serial.println(lan.ssid);
  } else if (p_cmd == "56") {
    lan.pwd = p_value;
    preferences.putString("p57", lan.pwd.c_str());
    Serial.print("pwd: ");
    Serial.println(lan.pwd);
  } else if (p_cmd == "99") {
    ESP.restart();
  }
  client.flush();
}


void udp_loop() {
  int packetSize = udp.parsePacket();
  if (packetSize) {
    //
    String pck = "";
    while (udp.available() > 0) {
      char c = udp.read();
      pck.concat(c);
    }
    udp.flush();
    Serial.print("pck: ");
    Serial.print(pck);
    f_proccess_data(pck[0], pck[1], pck[2], pck);
  }
}


void f_proccess_data(char c1, char c2, char c3, String pck) {
  String str = "";
  int year = 0;
  int month = 0;
  int day = 0;
  int hour = 0;
  int min = 0;
  int sec = 0;
  if (pck[0] == '0' && pck[1] == '0' && pck[2] == '0') {
    // 000 DADOS DA PLACA
    str = "0;";
    str.concat(etk.weight);
    str.concat(";");
    str.concat(etk.c_stable);
    str.concat(";");
    str.concat(etk.c_signal);
    str.concat(";");
    str.concat(digitalRead(IN1));
    str.concat(digitalRead(IN2));
    str.concat(digitalRead(IN3));
    str.concat(digitalRead(IN4));
    str.concat(";");
    str.concat(digitalRead(OUT1));
    str.concat(digitalRead(OUT2));
    str.concat(digitalRead(OUT3));
    str.concat(digitalRead(OUT4));
    str.concat(digitalRead(OUT5));
    str.concat(digitalRead(OUT6));
    str.concat(";0;0;0");
    str.concat(";");
    str.concat("0");
    sendUdp(str);
  } else if (pck[0] == '0' && pck[1] == '0' && pck[2] == '1') {
    // 001 RETORNA RFID
    str = "1;" + etk.rfidusr + ";" + etk.rfidprd;
    sendUdp(str);
  } else if (pck[0] == '0' && pck[1] == '0' && pck[2] == '2') {
    // 002 PEGA DATA E HORA
    str = "2;" + getDatetime();
    sendUdp(str);
  } else if (pck[0] == '0' && pck[1] == '0' && pck[2] == '3') {
    // 003 RESETA O ESP
    sendUdp("L;RESTART ESP");
    ESP.restart();
  } else if (pck[0] == '0' && pck[1] == '0' && pck[2] == '4') {
    f_serial_print("@ZER");
    sendUdp("L;@ZER");
  } else if (pck[0] == '0' && pck[1] == '0' && pck[2] == '5') {
    f_serial_print("@DES");
    sendUdp("L;@DES");
  } else if (pck[0] == '0' && pck[1] == '0' && pck[2] == '6') {
    f_serial_print("@PRI");
    sendUdp("L;@PRI");
  } else if (pck[0] == '0' && pck[1] == '0' && pck[2] == '7') {
    f_serial_print("@PES");
    sendUdp("L;@PES");
  } else if (pck[0] == '0' && pck[1] == '0' && pck[2] == '8') {
    f_serial_print("@SC0");
    sendUdp("L;@SC0");
  } else if (pck[0] == '0' && pck[1] == '0' && pck[2] == '9') {
    f_serial_print("@SC1");
    sendUdp("L;@SC1");
  } else if (pck[0] == '0' && pck[1] == '1' && pck[2] == '0') {
    f_serial_print("@SC2");
    sendUdp("L;@SC2");
  } else if (pck[0] == '0' && pck[1] == '1' && pck[2] == '1') {
    f_serial_print("@SC3");
    sendUdp("L;@SC3");
  } else if (pck[0] == '0' && pck[1] == '1' && pck[2] == '2') {
    f_serial_print("@SC4");
    sendUdp("L;@SC4");
  } else if (pck[0] == '0' && pck[1] == '1' && pck[2] == '3') {
    f_serial_print("@SC5");
    sendUdp("L;@SC5");
  } else if (pck[0] == '0' && pck[1] == '1' && pck[2] == '4') {
    f_serial_print("@SC6");
    sendUdp("L;@SC6");
  } else if (pck[0] == '0' && pck[1] == '1' && pck[2] == '5') {
    f_serial_print("@SC7");
    sendUdp("L;@SC7");
  } else if (pck[0] == '0' && pck[1] == '1' && pck[2] == '6') {
    f_serial_print("@SC8");
    sendUdp("L;@SC8");
  } else if (pck[0] == '0' && pck[1] == '1' && pck[2] == '7') {
    f_serial_print("@SC9");
    sendUdp("L;@SC9");
  } else if (pck[0] == '0' && pck[1] == '5' && pck[2] == '1') {
    if (digitalRead(OUT1) == LOW) {
      digitalWrite(OUT1, HIGH);
    } else {
      digitalWrite(OUT1, LOW);
    }
    sendUdp("L;@OUT1");
  } else if (pck[0] == '0' && pck[1] == '5' && pck[2] == '2') {
    if (digitalRead(OUT2) == LOW) {
      digitalWrite(OUT2, HIGH);
    } else {
      digitalWrite(OUT2, LOW);
    }
    sendUdp("L;@OUT2");
  } else if (pck[0] == '0' && pck[1] == '5' && pck[2] == '3') {
    if (digitalRead(OUT3) == LOW) {
      digitalWrite(OUT3, HIGH);
    } else {
      digitalWrite(OUT3, LOW);
    }
    sendUdp("L;@OUT3");
  } else if (pck[0] == '0' && pck[1] == '5' && pck[2] == '4') {
    if (digitalRead(OUT4) == LOW) {
      digitalWrite(OUT4, HIGH);
    } else {
      digitalWrite(OUT4, LOW);
    }
    sendUdp("L;@OUT4");
  } else if (pck[0] == '0' && pck[1] == '5' && pck[2] == '5') {
    if (digitalRead(OUT5) == LOW) {
      digitalWrite(OUT5, HIGH);
    } else {
      digitalWrite(OUT5, LOW);
    }
    sendUdp("L;@OUT5");
  } else if (pck[0] == '0' && pck[1] == '5' && pck[2] == '6') {
    if (digitalRead(OUT6) == LOW) {
      digitalWrite(OUT6, HIGH);
    } else {
      digitalWrite(OUT6, LOW);
    }
    sendUdp("L;@OUT6");


  } else if (pck[0] == '1' && pck[1] == '1' && pck[2] == '1') {
    // 111 ATUALIZA DATA E HORA
    String dt = "";
    dt.concat(pck[3]);
    dt.concat(pck[4]);
    dt.concat(pck[5]);
    dt.concat(pck[6]);
    int year = dt.toInt();
    dt = pck[7];
    dt.concat(pck[8]);
    int month = dt.toInt();
    dt = "";
    dt.concat(pck[9]);
    dt.concat(pck[10]);
    int day = dt.toInt();
    dt = "";
    dt.concat(pck[11]);
    dt.concat(pck[12]);
    int hour = dt.toInt();
    dt = "";
    dt.concat(pck[13]);
    dt.concat(pck[14]);
    int min = dt.toInt();
    dt = "";
    dt.concat(pck[15]);
    dt.concat(pck[16]);
    int sec = dt.toInt();
    rtc.adjust(DateTime(year, month, day, hour, min, sec));



  } else if (c1 == '3' && c2 == '9' && c3 == '9') {
    // 399 - COMANDO BUSCA DO VISOR
    sendUdp("399;" + mac_str + ";" + ETH.localIP().toString() + ";" + version + ";" + hversion);



  } else if (c1 == '4' && c2 == '0' && c3 == '0')
  // 400 BUSCA PARAMETROS LK2500
  {
    str = "400;";
    str.concat(etk.id);
    str.concat(";");
    str.concat(etk.apt_mode ? 1 : 0);
    str.concat(";");
    str.concat(etk.pwr_rfid ? 1 : 0);
    str.concat(";");
    str.concat(etk.portSerial);
    str.concat(";");
    str.concat(etk.portBaud);
    str.concat(";");
    str.concat(etk.w_capacity);
    str.concat(";");
    str.concat(etk.w_min);
    str.concat(";");
    str.concat(etk.w_max);
    str.concat(";");
    str.concat(etk.w_variation);
    str.concat(";");
    str.concat(etk.productcode);
    str.concat(";");
    str.concat(etk.qtd_w_apt);
    sendUdp(str);

  } else if (c1 == '4' && c2 == '0' && c3 == '1')
  // 401 grava id
  {
    etk.id = pck.substring(3).toInt();
    preferences.putInt("p0", etk.id);
  } else if (c1 == '4' && c2 == '0' && c3 == '2')
  // 402 grava modo apontamento
  {
    etk.apt_mode = (pck[3] == '1') ? true : false;
    preferences.putBool("p6", etk.apt_mode);
  } else if (c1 == '4' && c2 == '0' && c3 == '3')
  // 403 grava tipo inicializacao
  {
    etk.pwr_rfid = (pck[3] == '1') ? true : false;
    preferences.putBool("p1", etk.pwr_rfid);
  } else if (c1 == '4' && c2 == '0' && c3 == '4')
  // 404 grava serial
  {
    etk.portSerial = pck.substring(3).toInt();
    preferences.putShort("p8", etk.portSerial);
  } else if (c1 == '4' && c2 == '0' && c3 == '5')
  // 405 grava baud
  {
    etk.portBaud = pck.substring(3).toInt();
    preferences.putInt("p9", etk.portBaud);
  } else if (c1 == '4' && c2 == '0' && c3 == '6')
  // 406 grava capacidade
  {
    etk.w_capacity = pck.substring(3).toInt();
    preferences.putInt("p4", etk.w_capacity);
  } else if (c1 == '4' && c2 == '0' && c3 == '7')
  // 407 grava peso minimo
  {
    etk.w_min = pck.substring(3).toInt();
    preferences.putInt("p2", etk.w_min);
  } else if (c1 == '4' && c2 == '0' && c3 == '8')
  // 408 grava peso maximo
  {
    etk.w_max = pck.substring(3).toInt();
    preferences.putInt("p3", etk.w_max);
  } else if (c1 == '4' && c2 == '0' && c3 == '9')
  // 409 grava variacao
  {
    etk.w_variation = pck.substring(3).toInt();
    preferences.putInt("p5", etk.w_variation);
  } else if (c1 == '4' && c2 == '1' && c3 == '0')
  // 410 grava produto
  {
    etk.productcode = pck.substring(3);
    preferences.putString("p7", etk.productcode);
  } else if (c1 == '4' && c2 == '1' && c3 == '1')
  // 411 grava pesagens para apontamento
  {
    etk.qtd_w_apt = pck.substring(3).toInt();
    preferences.putShort("p10", etk.qtd_w_apt);



  } else if (c1 == '5' && c2 == '0' && c3 == '0')
  // 500 BUSCA PARAMETROS DE REDE
  {
    str = "500;";
    str.concat(lan.wifi ? 1 : 0);
    str.concat(";");
    str.concat(lan.dhcp ? 1 : 0);
    str.concat(";");
    str.concat(lan.ip);
    str.concat(";");
    str.concat(lan.msk);
    str.concat(";");
    str.concat(lan.dns);
    str.concat(";");
    str.concat(lan.gtw);
    str.concat(";");
    str.concat(lan.ssid);
    str.concat(";");
    str.concat(lan.pwd);
    str.concat(";");
    str.concat(lan.use_tcp ? 1 : 0);
    str.concat(";");
    str.concat(lan.tcp_hip);
    str.concat(";");
    str.concat(lan.tcp_hport);
    sendUdp(str);

  } else if (c1 == '5' && c2 == '0' && c3 == '1')
  // 501 grava tipo rede
  {
    lan.wifi = (pck[3] == '1') ? true : false;
    preferences.putBool("p50", lan.wifi);
  } else if (c1 == '5' && c2 == '0' && c3 == '2')
  // 502 grava tipo conexao
  {
    lan.dhcp = (pck[3] == '1') ? true : false;
    preferences.putBool("p51", lan.dhcp);
  } else if (c1 == '5' && c2 == '0' && c3 == '3')
  // 503 grava ip
  {
    lan.ip = pck.substring(3);
    preferences.putString("p52", lan.ip);
    sendUdp("L;update ip");
  } else if (c1 == '5' && c2 == '0' && c3 == '4')
  // 504 grava msk
  {
    lan.msk = pck.substring(3);
    preferences.putString("p53", lan.msk);
    sendUdp("L;update msk");
  } else if (c1 == '5' && c2 == '0' && c3 == '5')
  // 505 grava msk
  {
    lan.dns = pck.substring(3);
    preferences.putString("p55", lan.dns);
    sendUdp("L;update dns");
  } else if (c1 == '5' && c2 == '0' && c3 == '6')
  // 506 grava msk
  {
    lan.gtw = pck.substring(3);
    preferences.putString("p54", lan.gtw);
    sendUdp("L;update gtw");
  } else if (c1 == '5' && c2 == '0' && c3 == '7')
  // 507 grava msk
  {
    lan.ssid = pck.substring(3);
    preferences.putString("p56", lan.ssid);
    sendUdp("L;update ssid");
  } else if (c1 == '5' && c2 == '0' && c3 == '8')
  // 508 grava msk
  {
    lan.pwd = pck.substring(3);
    preferences.putString("p57", lan.pwd);
    sendUdp("L;update pwd");
  } else if (c1 == '5' && c2 == '0' && c3 == '9')
  // 509 grava msk
  {
    lan.use_tcp = (pck[3] == '1') ? true : false;
    preferences.putBool("p60", lan.use_tcp);
    sendUdp("L;update use_tcp");
  } else if (c1 == '5' && c2 == '1' && c3 == '0')
  // 510 grava host ip
  {
    lan.tcp_hip = pck.substring(3);
    preferences.putString("p58", lan.tcp_hip);
    sendUdp("L;update tcp_hip");
  } else if (c1 == '5' && c2 == '1' && c3 == '1')
  // 511 grava host port
  {
    lan.tcp_hport = pck.substring(3).toInt();
    preferences.putUInt("p59", lan.tcp_hport);
    sendUdp("L;update tcp_hport");
  } else if (c1 == '8' && c2 == '8' && c3 == '8') {
    // 888 - LISTAR DADOS DOS ARQUIVOS
    str = readFile(SPIFFS, pck.substring(3).c_str());
    if (str) {
      String r1 = ";";
      String r2 = "_";
      str.replace(r1, r2);
      sendUdp("L;" + str);
    }
  } else if (c1 == '9' && c2 == '9' && c3 == '6') {
    // 996 SETA ID DA ESTACAO
    etk.id = pck.substring(3).toInt();
    preferences.putInt("p0", etk.id);
  } else if (c1 == '9' && c2 == '9' && c3 == '8') {
    // 998 RETORA O MAC ADDRESS
    sendUdp("998;" + mac_str);
  } else if (c1 == '9' && c2 == '9' && c3 == '9') {
    // 999 RETORNA A VERSAO
    sendUdp("999;" + version);
  }
}


void f_preferences() {
  preferences.begin("etk", false);
  delay(200);
  // se for a primeira vez que estiver rodando entao limpa a preferences e recria
  short frun = preferences.getBool("frun", true);
  if (frun) {
    // limpa todos os dados de preferencias
    preferences.clear();
    // param etk
    preferences.putBool("frun", false);
    etk.id = 200;
    preferences.putInt("p0", etk.id);
    etk.pwr_rfid = false;
    preferences.putBool("p1", etk.pwr_rfid);
    etk.w_min = 200;
    preferences.putInt("p2", etk.w_min);
    etk.w_max = 6000;
    preferences.putInt("p3", etk.w_max);
    etk.w_capacity = 6000;
    preferences.putInt("p4", etk.w_capacity);
    etk.w_variation = 10;
    preferences.putInt("p5", etk.w_variation);
    etk.apt_mode = false;
    preferences.putBool("p6", etk.apt_mode);
    etk.productcode = "";
    preferences.putString("p7", etk.productcode);
    etk.portSerial = 0;
    preferences.putShort("p8", etk.portSerial);
    etk.portBaud = 9600;
    preferences.putInt("p9", etk.portBaud);
    etk.qtd_w_apt = 1;
    preferences.putShort("p10", etk.qtd_w_apt);
    // valores REDE
    lan.wifi = true;
    preferences.putBool("p50", lan.wifi);
    lan.dhcp = true;
    preferences.putBool("p51", lan.dhcp);
    lan.ip = "192.168.4.2";
    preferences.putString("p52", lan.ip);
    lan.msk = "255.255.255.0";
    preferences.putString("p53", lan.msk);
    lan.gtw = "192.168.4.1";
    preferences.putString("p54", lan.gtw);
    lan.dns = "192.168.4.1";
    preferences.putString("p55", lan.dns);
    lan.ssid = "lenkelog";
    preferences.putString("p56", lan.ssid);
    lan.pwd = "38844266";
    preferences.putString("p57", lan.pwd);
    lan.tcp_hip = "192.168.4.1";
    preferences.putString("p58", lan.tcp_hip);
    lan.tcp_hport = 13085;
    preferences.putUInt("p59", lan.tcp_hport);
    lan.use_tcp = false;
    preferences.putBool("p60", lan.use_tcp);
  } else {
    // param etk
    etk.id = preferences.getInt("p0", 1);
    etk.pwr_rfid = preferences.getBool("p1", false);
    etk.w_min = preferences.getInt("p2", 200);
    etk.w_max = preferences.getInt("p3", 6000);
    etk.w_capacity = preferences.getInt("p4", 6000);
    etk.w_variation = preferences.getInt("p5", 10);
    etk.apt_mode = preferences.getBool("p6", false);
    etk.productcode = preferences.getString("p7", "0");
    etk.portSerial = preferences.getShort("p8", 0);
    etk.portBaud = preferences.getInt("p9", 9600);
    etk.qtd_w_apt = preferences.getShort("p10", 1);
    //
    // param network
    lan.wifi = preferences.getBool("p50", true);
    lan.dhcp = preferences.getBool("p51", true);
    lan.ip = preferences.getString("p52", "192.168.4.2");
    lan.msk = preferences.getString("p53", "255.255.255.0");
    lan.gtw = preferences.getString("p54", "192.168.4.1");
    lan.dns = preferences.getString("p55", "192.168.4.1");
    lan.ssid = preferences.getString("p56", "lenkelog");
    lan.pwd = preferences.getString("p57", "38844266");
    lan.tcp_hip = preferences.getString("p58", "192.168.4.1");
    lan.tcp_hport = preferences.getUInt("p59", 13085);
    lan.use_tcp = preferences.getBool("p60", false);
  }
  //
}


String getDatetime() {
  DateTime now = rtc.now();
  char newDt[25] = { 0 };
  sprintf(newDt, "%04u-%02u-%02u %02u:%02u:%02u", now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());
  return String(newDt);
}


void f_readMacAddress() {
  char baseMacChr[18] = { 0 };
  uint8_t baseMac[8];
  esp_efuse_mac_get_default(baseMac);
  mac[0] = baseMac[0];
  mac[1] = baseMac[1];
  mac[2] = baseMac[2];
  mac[3] = baseMac[3];
  mac[4] = baseMac[4];
  mac[5] = baseMac[5];
  //Serial.printf("%02x:%02x:%02x:%02x:%02x:%02x\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  sprintf(baseMacChr, "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  mac_str = String(baseMacChr);
}

//------------------------------------
//-- SPIFFSS
//------------------------------------
void f_iniSpiffs() {
  if (!SPIFFS.begin(true)) {
    Serial.println("Um erro ocorreu durente a montagem do SPIFFS");
  }
  Serial.println("SPIFFS montado com sucesso");
}

String getNextFile(fs::FS &fs) {
  File root = fs.open("/");
  File file = root.openNextFile();
  while (file) {
    String fileName = file.name();
    if (fileName.substring(0, 3) == "apt") {
      Serial.println("ENCONTRADO ARQUIVO: " + fileName);
      return "/" + fileName;
    }
    file = root.openNextFile();
  }
  return "";
}

void deleteFile(fs::FS &fs, const char *path) {
  Serial.printf("removendo arquivo: %s\r\n", path);
  fs.remove(path);
}

String readFile(fs::FS &fs, const char *path) {
  Serial.printf("\r\nLendo arquivo: %s\r\n", path);
  File file = fs.open(path);
  if (!file || file.isDirectory()) {
    Serial.println("- failed to open file for reading");
    return String();
  }
  String fileContent;
  while (file.available()) {
    fileContent = file.readStringUntil('\n');
    break;
  }
  return fileContent;
}

void writeFile(fs::FS &fs, const char *path, const char *message) {
  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("- falhou ao abrir o arquivo para escrita");
    return;
  }
  if (file.print(message)) {
    Serial.println("- escrevendo arquivo.");
  } else {
    Serial.println("- falhou ar escrever no arquivo");
  }
}

void createFileApt(String str) {
  Serial.println("SALVANDO PESO: " + str);
  DateTime now = rtc.now();
  String path = "/apt" + String(now.year(), DEC) + String(now.month(), DEC) + String(now.day(), DEC) + String(now.hour(), DEC) + String(now.minute(), DEC) + String(now.second(), DEC) + ".txt";
  Serial.println(path);
  writeFile(SPIFFS, path.c_str(), str.c_str());
}
