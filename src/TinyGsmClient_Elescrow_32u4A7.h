/**
 * @file       TinyGsmClient_Elescrow_32u4A7.h TinyGsmClient library neede for proper functioning
 * @author     Volodymyr Shymanskyy - R.L mod.
 * @license    LGPL-3.0
 * @copyright  Copyright (c) 2016 Volodymyr Shymanskyy - modyfied by R.L. for 32u4 with A7 GPRS/GSM
 * @date       Jun 2017
 */

#pragma once

//#ifndef TinyGsmClient_Elescrow_32u4A7_h
//#define TinyGsmClient_h
//#define TinyGsmClient_Elescrow_32u4A7_h
//#define TINY_GSM_DEBUG Serial

#if !defined(TINY_GSM_RX_BUFFER)
  #define TINY_GSM_RX_BUFFER 256
#endif
//period of data prepared by A7 module GPS //TO DO BY RL
#if !defined(A7_GPS_PERIOD)
  #define A7_GPS_PERIOD 1
#endif

#define APN "wap.tim.it"
#define USER "WAPTIM"
#define PASS "WAPTIM"

#define MQTT_SERVER "example.com"
#define MQTT_USER "mttuser"
#define MQTT_PASSWORD "mqttpassword"

#define TRACCAR_SERVER "traccar.example.com"
#define TRACCAR_ID "123"

#include "TinyGsmCommon.h"

#define GSM_NL "\r\n"
static const char GSM_OK[] TINY_GSM_PROGMEM = "OK" GSM_NL;
static const char GSM_ERROR[] TINY_GSM_PROGMEM = "ERROR" GSM_NL;
#define ss Serial1


int errSendCoordinate = 0;

enum SimStatus {
  SIM_ERROR = 0,
  SIM_READY = 1,
  SIM_LOCKED = 2,
};

enum RegStatus {
  REG_UNREGISTERED = 0,
  REG_SEARCHING    = 2,
  REG_DENIED       = 3,
  REG_OK_HOME      = 1,
  REG_OK_ROAMING   = 5,
  REG_UNKNOWN      = 4,
};


class TinyGsm
{

public:
  TinyGsm(Stream& stream)
    : stream(stream)
  {}

public:

class GsmClient : public Client
{
  friend class TinyGsm;
  typedef TinyGsmFifo<uint8_t, TINY_GSM_RX_BUFFER> RxFifo;

public:
  GsmClient() {}

  GsmClient(TinyGsm& modem) {
    init(&modem);
  }

  bool init(TinyGsm* modem) {

    //pinMode(4, OUTPUT);
    // pinMode(5, OUTPUT);
    //pinMode(8, OUTPUT);
    //digitalWrite(5, HIGH);
    //digitalWrite(4, LOW);
    //digitalWrite(8, LOW);
    DDRD |= (1<<PD4);  // pin 4 PD4 output
    DDRC |= (1<<PC6);//pin 5 PC6 output
    DDRB |= (1<<PB4); //pin 8 PB4 output
    PORTC |= (1<<PC6); //digitalWrite(5, HIGH);
    PORTD &= ~(1 << PD4); //pin 4 PD4 Low //digitalWrite(4, LOW);
    PORTB &= ~(1 << PB4); //pin 8 PB4 LOW //digitalWrite(8, LOW);





    this->at = modem;
    this->mux = -1;
    sock_connected = false;

    return true;
  }

public:
  virtual int connect(const char *host, uint16_t port) {
    TINY_GSM_YIELD();
    rx.clear();
    uint8_t newMux = -1;
    sock_connected = at->modemConnect(host, port, &newMux);
    if (sock_connected) {
      mux = newMux;
      at->sockets[mux] = this;
    }
    return sock_connected;
  }

  virtual int connect(IPAddress ip, uint16_t port) {
    String host; host.reserve(16);
    host += ip[0];
    host += ".";
    host += ip[1];
    host += ".";
    host += ip[2];
    host += ".";
    host += ip[3];
    return connect(host.c_str(), port);
  }

  virtual void stop() {
    TINY_GSM_YIELD();
    at->sendAT(GF("+CIPCLOSE="), mux);
    sock_connected = false;
    at->waitResponse();
  }

  virtual size_t write(const uint8_t *buf, size_t size) {
    TINY_GSM_YIELD();
    //at->maintain();
    return at->modemSend(buf, size, mux);
    at->waitResponse();
  }

  virtual size_t write(uint8_t c) {
    return write(&c, 1);
  }

  virtual int available() {
    TINY_GSM_YIELD();
    if (!rx.size()) {
      at->maintain();
    }
    return rx.size();
  }

  virtual int read(uint8_t *buf, size_t size) {
    TINY_GSM_YIELD();
    size_t cnt = 0;
    while (cnt < size) {
      size_t chunk = TinyGsmMin(size-cnt, rx.size());
      if (chunk > 0) {
        rx.get(buf, chunk);
        buf += chunk;
        cnt += chunk;
        continue;
      }
      // TODO: Read directly into user buffer?
      if (!rx.size()) {
        at->maintain();
        //break;
      }
    }
    return cnt;
  }

  virtual int read() {
    uint8_t c;
    if (read(&c, 1) == 1) {
      return c;
    }
    return -1;
  }

  virtual int peek() { return -1; } //TODO
  virtual void flush() { at->stream.flush(); }

  virtual uint8_t connected() {
    if (available()) {
      return true;
    }
    return sock_connected;
  }
  virtual operator bool() { return connected(); }
private:
  TinyGsm*      at;
  uint8_t       mux;
  bool          sock_connected;
  RxFifo        rx;
};

public:

  /*
   * Basic functions
   */
  bool begin() {
    return init();
  }

  bool init() {
    if (!autoBaud()) {
      return false;
    }
    /*sendAT(GF("&FZE0"));  // Factory + Reset + Echo Off
    if (waitResponse() != 1) {
      return false;
    }
    sendAT(GF("+CMEE=0"));
    waitResponse();*/

    getSimStatus();
    return true;
  }

  bool autoBaud(unsigned long timeout = 10000L) {
    for (unsigned long start = millis(); millis() - start < timeout; ) {
      sendAT(GF("E0"));
      if (waitResponse(200) == 1) {
          delay(100);
          return true;
      }
      delay(100);
    }
    return false;
  }

  void maintain() {
    //while (stream.available()) {
      waitResponse(10, NULL, NULL);
    //}
  }

  bool factoryDefault() {
    sendAT(GF("&FZE0&W"));  // Factory + Reset + Echo Off + Write
    waitResponse();
    sendAT(GF("&W"));       // Write configuration
    return waitResponse() == 1;
  }

  /*
   * Power functions
   */

  bool restart() {
    if (!autoBaud()) {
      return false;
    }
    //sendAT(GF("+GPS=0"));
    Serial.println("Restart A9!");
    stopGPS();
    sendAT(GF("+RST=1"));
    delay(30000);


    return true;
  }

  /*
   * SIM card & Networ Operator functions
   */

  bool simUnlock(const char *pin) {
    sendAT(GF("+CPIN=\""), pin, GF("\""));
    return waitResponse() == 1;
  }

  String getSimCCID() {
    sendAT(GF("+CCID"));
    if (waitResponse(GF(GSM_NL "+ICCID:")) != 1) {
      return "";
    }
    String res = stream.readStringUntil('\n');
    waitResponse();
    res.trim();
    return res;
  }

  String getIMEI() {
    sendAT(GF("+GSN"));
    if (waitResponse(GF(GSM_NL)) != 1) {
      return "";
    }
    String res = stream.readStringUntil('\n');
    waitResponse();
    res.trim();
    return res;
  }

  int getSignalQuality() {
    sendAT(GF("+CSQ"));
    if (waitResponse(GF(GSM_NL "+CSQ:")) != 1) {
      return 99;
    }
    int res = stream.readStringUntil(',').toInt();
    waitResponse();
    return res;
  }

  bool callAnswer() {
    sendAT(GF("A"));
    return waitResponse() == 1;
  }

  bool callNumber(const String& number) {
    sendAT(GF("D"), number);
    return waitResponse() == 1;
  }

  bool callHangup(const String& number) {
    sendAT(GF("H"), number);
    return waitResponse() == 1;
  }

  bool sendSMS(const String& number, const String& text) {
    sendAT(GF("+CMGF=1"));
    waitResponse();
    sendAT(GF("+CMGS=\""), number, GF("\""));
    if (waitResponse(GF(">")) != 1) {
      return false;
    }
    stream.print(text);
    stream.write((char)0x1A);
    return waitResponse(60000L) == 1;
  }

  SimStatus getSimStatus(unsigned long timeout = 10000L) {
    for (unsigned long start = millis(); millis() - start < timeout; ) {
      sendAT(GF("+CPIN?"));
      if (waitResponse(GF(GSM_NL "+CPIN:")) != 1) {
        delay(1000);
        continue;
      }
      int status = waitResponse(GF("READY"), GF("SIM PIN"), GF("SIM PUK"));
      waitResponse();
      switch (status) {
      case 2:
      case 3:  return SIM_LOCKED;
      case 1:  return SIM_READY;
      default: return SIM_ERROR;
      }
    }
    return SIM_ERROR;
  }

  RegStatus getRegistrationStatus() {
    sendAT(GF("+CREG?"));
    if (waitResponse(GF(GSM_NL "+CREG:")) != 1) {
      return REG_UNKNOWN;
    }
    streamSkipUntil(','); // Skip format (0)
    int status = stream.readStringUntil('\n').toInt();
    waitResponse(10000L);
    return (RegStatus)status;
  }

  String getOperator() {

    sendAT(GF("+COPS=3,0"));
    waitResponse(1000L);
    //sendAT(GF("&W"));
    //waitResponse(1000L);
    sendAT(GF("+COPS?"));

    if (waitResponse(GF(GSM_NL "+COPS:")) != 1) {
      return "";
     }
    streamSkipUntil('"'); // Skip mode and format
    String res = stream.readStringUntil('"');
    waitResponse(1000L);
    return res;
  }

  bool waitForNetwork(unsigned long timeout = 30000L) {
    for (unsigned long start = millis(); millis() - start < timeout; ) {
      RegStatus s = getRegistrationStatus();
      if (s == REG_OK_HOME || s == REG_OK_ROAMING) {
        return true;
      } else if (s == REG_UNREGISTERED || s == REG_UNKNOWN || s == REG_DENIED) {
        return false;
      }
      delay(1000);
    }
    return false;
  }

  /*
   * GPRS functions
   */
    //bool gprsConnect(const char* apn, const char* user, const char* pwd) {
    bool gprsConnect(const __FlashStringHelper*  apn, const __FlashStringHelper*  user, const __FlashStringHelper*  pwd) {
    gprsDisconnect();

    Serial.println(F("Sending CGATT"));
    sendAT(GF("+CGATT=1"));
    if (waitResponse(10000L) != 1)
      return false;

      Serial.println(F("Sending +CSTT"));
      sendAT(GF("+CSTT=\""), apn, GF("\",\""), user, GF("\",\""), pwd, GF("\""));
      if (waitResponse(10000L) != 1) {
        return false;
      }

    Serial.println(F("Sending +CIICR"));
    sendAT(GF("+CIICR"));
    waitResponse();
    Serial.println(F("Sending +CIFSR"));
    sendAT(GF("+CIFSR"));
    waitResponse();

    sendAT(GF("+MQTTCONN="),MQTT_SERVER,GF("\",1883,\""),MQTT_USER,GF("\",120,0,\""),MQTT_USER,GF("\",\""),MQTT_PASSWORD,GF("\""));
    waitResponse(10000L);
    return true;
  }

  bool gprsDisconnect() {
    sendAT(GF("+CIPSHUT"));
    return waitResponse(6000L) == 1;
  }

  /*
   * Phone Call functions
   */

  /*
   * Messaging functions
   */

  void sendUSSD() {
  }

  void sendSMS() {
  }

  /*
   * Location functions
   */
  String getLocation() {

    sendAT(GF("+LOCATION=2"));
stream.readStringUntil('\n');
    String lat= stream.readStringUntil(',');
    String lon = stream.readStringUntil('\n');

    //String res = stream.readStringUntil('\n');
    waitResponse(5000L);
    //res.trim();
    String position = lat+','+lon;
    position.trim();
    position.replace("\n","");
    position.replace("\r","");
    position.replace("OK","");
    //Serial.print("RAW:");
    //Serial.println(position);
    //Serial.println(position.indexOf(",0."));
    if(position.indexOf(",0.") == -1){
      return position;
    }

    return "INVALID";
  }


    /*
   * Location start
   */

  void startGPS() {
    Serial.println("Start gps..");

    //sendAT(GF("AT"));
    //waitResponse(500L);

    sendAT(GF("+GPS=0"));

    if(waitResponse(5000L)!=1){
      errSendCoordinate++;
    }

    sendAT(GF("+GPSMD=2"));
    if(waitResponse(5000L)!=1){
      errSendCoordinate++;
    }


    sendAT(GF("+GPS=1"));
    if(waitResponse(5000L)!=1){
      errSendCoordinate++;
    }

    sendAT(GF("+GPSRD="),0);
    if(waitResponse(5000L)!=1){
      errSendCoordinate++;
    }


    sendAT(GF("+GPSLP=0"));
    if(waitResponse(5000L)!=1){
      errSendCoordinate++;
    }
  }

  /*
   * Location start
   */
  void stopGPS() {

    Serial.println("Stop gps.. disabled");
    /*sendAT(GF("+GPS=0"));
    waitResponse(10000L);

    sendAT(GF("+GPSRD=0"));
    waitResponse(60000L);*/
  }

    void stopsendingGPS() {

    //sendAT(GF("AT"));
    //waitResponse(00L);

    //sendAT(GF("+GPS=0"));
    //waitResponse(60000L);

    sendAT(GF("+GPSRD=0"));
    waitResponse(6000L);
  }

    void startsendingGPS() {


    //sendAT(GF("AT"));
    //waitResponse(500L);

    //sendAT(GF("+GPS=0"));
    //waitResponse(60000L);

    //sendAT(GF("+GPS=1"));
    //waitResponse(60000L);

    sendAT(GF("+GPSRD="),0);
    waitResponse(10000L);
  }

  /*
   * Battery functions
   */

    uint8_t getBattVoltage() {
    //sendAT(GF("AT"));
    //waitResponse(500L);

    sendAT(GF("+CBC?"));
    //if (waitResponse(GF("+CBC:")) != 1) {
    //  return 0;
    //}

    streamSkipUntil(' '); // Skip
    streamSkipUntil(','); // Skip

    uint8_t res = stream.readStringUntil('\n').toInt();
    waitResponse(10000L);

    return res;
  }

    uint16_t getBattVoltagemv(){

            //map(double value, float x_min, float x_max, float y_min, float y_max)
           //x_min=0,x_max=100,y_min=3200,y_max=4200
      return (3200 + (((4200 - 3200)/(100 - 0)) * (getBattVoltage() - 0)));
   }

   float getExtBatteryVoltage(){
     int sensorValue = analogRead(A0);
      float voltage = sensorValue * 0.02313320826;//(16.5 / 1023.0);
      Serial.println(sensorValue);
     return voltage;
   }



private:
  int modemConnect(const char* host, uint16_t port, uint8_t* mux) {
    sendAT(GF("+CIPSTART="),  GF("\"TCP"), GF("\",\""), host, GF("\","), port);

    if (waitResponse(75000L, GF(GSM_NL "+CIPNUM:")) != 1) {
      return -1;
    }
    int newMux = stream.readStringUntil('\n').toInt();

    int rsp = waitResponse(60000L,
                           GF("CONNECT OK" GSM_NL),
                           GF("CONNECT FAIL" GSM_NL),
                           GF("ALREADY CONNECT" GSM_NL));
    if (waitResponse() != 1) {
      return -1;
    }
    *mux = newMux;

    return (1 == rsp);
  }

  public: bool ping(){
    //Serial.println("pinging the modem...");
    sendAT(GF(""));
    return waitResponse(5000L) == 1;
  }




    public: void sendCoordinateMQTT(String dataToSend){
    Serial.println("sending mqtt...");
    //dataToSend.replace("'", '\"');
    sendAT(GF("+MQTTPUB=\"owntracks/gpstracker/arduino\",\""),dataToSend,GF("\",0,0,0"));
    //waitResponse(20000L);

    Serial.println(dataToSend);

    if(waitResponse(20000)!=1){

      Serial.print("Errore Invio Coordinate con mqtt");
      sendAT(GF("+MQTTCONN="),MQTT_SERVER,GF("\",1883,\""),MQTT_USER,GF("\",120,0,\""),MQTT_USER,GF("\",\""),MQTT_PASSWORD,GF("\""));
      waitResponse(10000L);
      errSendCoordinate++;
      Serial.println(errSendCoordinate);
    }else{
      errSendCoordinate=0;
      //waitResponse(5000L, GF(GSM_NL "content-length: 0"));
      Serial.println("mqtt message sent!");
      while(!ping()){
          Serial.println("waiting modem alive..");
      }
    }

    if(errSendCoordinate >0){
      Serial.println("********** Riavvio per troppi errori consecutivi... **********");
      while(!ping()){
          Serial.println("waiting modem alive before restart..");
      }
      restart();

      int triesSim = 0;
      while(getSimStatus() != 1){
        while(!ping()){
            Serial.println("waiting modem alive..");
        }
        Serial.println("waiting sim ready..");
        triesSim ++;
        if (triesSim>1){
          Serial.println("Restarting Again..");
          triesSim = 0;
          restart();
        }
      }
      int networkTries = 0;
      while(!waitForNetwork()){
          Serial.println("Wait network to be ready...");
          delay(1000L);
          networkTries++;
          if (networkTries > 60){
            networkTries = 0;
            restart();
            while(!ping()){
                Serial.println("Wait modem to be ready...");
            }
          }
      }
      gprsConnect(F(APN), F(USER), F(PASS));

      waitResponse(10000L);
      startGPS();
    }
  }

  public: void sendCoordinate(String dataToSend){
    Serial.println("sending...");
    Serial.println(dataToSend);

    sendAT(GF("+HTTPGET=\"http://"),TRACCAR_SERVER,GF(":5055/?id="),TRACCAR_ID,GF("&"),dataToSend,GF("\""));

    if(waitResponse(20000)!=1){

      Serial.print("Errore Invio Coordinate");
      errSendCoordinate++;
      Serial.println(errSendCoordinate);
    }else{
      errSendCoordinate=0;
      waitResponse(5000L, GF(GSM_NL "content-length: 0"));
      Serial.println("sent!");
      while(!ping()){
          Serial.println("waiting modem alive..");
      }
    }

    if(errSendCoordinate >0){
      Serial.println("********** Riavvio per troppi errori consecutivi... **********");
      while(!ping()){
          Serial.println("waiting modem alive before restart..");
      }
      restart();

      int triesSim = 0;
      while(getSimStatus() != 1){
        while(!ping()){
            Serial.println("waiting modem alive..");
        }
        Serial.println("waiting sim ready..");
        triesSim ++;
        if (triesSim>1){
          Serial.println("Restarting Again..");
          triesSim = 0;
          restart();
        }
      }
      int networkTries = 0;
      while(!waitForNetwork()){
          Serial.println("Wait network to be ready...");
          delay(1000L);
          networkTries++;
          if (networkTries > 60){
            networkTries = 0;
            restart();
            while(!ping()){
                Serial.println("Wait modem to be ready...");
            }
          }
      }
      gprsConnect(F(APN), F(USER), F(PASS));
      startGPS();
    }

    /*owntrackData.replace('"', '\'');
    sendAT(GF("+MQTTPUB=\"owntracks/gpstracker/arduino\",\""),owntrackData,GF("\",0,0,1"));
    waitResponse();
    Serial.println(owntrackData);
    Serial.println("+MQTTPUB=\"owntracks/gpstracker/arduino\",\""+owntrackData+"\",0,0,1");*/
    //waitResponse();
  }

  int modemSend(const void* buff, size_t len, uint8_t mux) {
    sendAT(GF("+CIPSEND="), mux, ',', len);
    //TINY_GSM_YIELD();
    //waitResponse();

    if (waitResponse(15000L, GF(GSM_NL ">")) != 1) {
      return -1;
    }
    stream.write((uint8_t*)buff, len);
    //TINY_GSM_YIELD();
    //waitResponse();
    stream.flush();
    stream.flush();
    if (waitResponse(10000L, GFP(GSM_OK), GF(GSM_NL "FAIL")) != 1) {
      return -1;
    }
    return len;
  }

  bool modemGetConnected(uint8_t mux) { //TODO mux?
    sendAT(GF("+CIPSTATUS"));
    int res = waitResponse(GF(",\"CONNECTED\""), GF(",\"CLOSED\""), GF(",\"CLOSING\""), GF(",\"INITIAL\""));

    waitResponse();
    return 1 == res;
  }

  /* Utilities */
  template<typename T>
  void streamWrite(T last) {

    stream.print(last);
  }

  template<typename T, typename... Args>
  void streamWrite(T head, Args... tail) {
    stream.print(head);
    streamWrite(tail...);
  }

  int streamRead() { return stream.read(); }

  bool streamSkipUntil(char c) { //TODO: timeout
    while (true) {
      while (!stream.available()) {}
      if (stream.read() == c)
        return true;
    }
    return false;
  }

  template<typename... Args>
  void sendAT(Args... cmd) {
    streamWrite("AT", cmd..., GSM_NL);
    stream.flush();
    TINY_GSM_YIELD();
    //DBG("### AT:", cmd...);
  }


  // TODO: Optimize this!
  uint8_t waitResponse(uint32_t timeout, String& data,
                       GsmConstStr r1=GFP(GSM_OK), GsmConstStr r2=GFP(GSM_ERROR),
                       GsmConstStr r3=NULL, GsmConstStr r4=NULL, GsmConstStr r5=NULL)
  {
    /*String r1s(r1); r1s.trim();
    String r2s(r2); r2s.trim();
    String r3s(r3); r3s.trim();
    String r4s(r4); r4s.trim();
    String r5s(r5); r5s.trim();
    DBG("### ..:", r1s, ",", r2s, ",", r3s, ",", r4s, ",", r5s);*/
    data.reserve(64);
    int index = 0;
    unsigned long startMillis = millis();
    do {
      TINY_GSM_YIELD();
      while (stream.available() > 0) {
        int a = streamRead();
        if (a <= 0) continue; // Skip 0x00 bytes, just in case
        data += (char)a;
        if (r1 && data.endsWith(r1)) {
          index = 1;
          goto finish;
        } else if (r2 && data.endsWith(r2)) {
          index = 2;
          goto finish;
        } else if (r3 && data.endsWith(r3)) {
          index = 3;
          goto finish;
        } else if (r4 && data.endsWith(r4)) {
          index = 4;
          goto finish;
        } else if (r5 && data.endsWith(r5)) {
          index = 5;
          goto finish;
        } else if (data.endsWith(GF("+CIPRCV:"))) {
          int mux = stream.readStringUntil(',').toInt();
          int len = stream.readStringUntil(',').toInt();
          if (len > sockets[mux]->rx.free()) {
            DBG("### Buffer overflow: ", len, "->", sockets[mux]->rx.free());
          } else {
            DBG("### Got: ", len, "->", sockets[mux]->rx.free());
          }
          while (len--) {
            while (!stream.available()) {}
            sockets[mux]->rx.put(stream.read());
          }
          data = "";
          return index;
        } else if (data.endsWith(GF("+TCPCLOSED:"))) {
          int mux = stream.readStringUntil(',').toInt();
          stream.readStringUntil('\n');
          sockets[mux]->sock_connected = false;
          data = "";
        }
      }
    } while (millis() - startMillis < timeout);
finish:
    if (!index) {
      data.trim();
      if (data.length()) {
        Serial.print("### Unhandled:");
        Serial.print(data);
        DBG("### Unhandled:", data);
      }
      data = "";
    }
    //Serial.println(data);
    return index;
  }

  uint8_t waitResponse(uint32_t timeout,
                       GsmConstStr r1=GFP(GSM_OK), GsmConstStr r2=GFP(GSM_ERROR),
                       GsmConstStr r3=NULL, GsmConstStr r4=NULL, GsmConstStr r5=NULL)
  {
    String data;
    return waitResponse(timeout, data, r1, r2, r3, r4, r5);
  }

  uint8_t waitResponse(GsmConstStr r1=GFP(GSM_OK), GsmConstStr r2=GFP(GSM_ERROR),
                       GsmConstStr r3=NULL, GsmConstStr r4=NULL, GsmConstStr r5=NULL)
  {
    return waitResponse(1000, r1, r2, r3, r4, r5);
  }

private:
  Stream&       stream;
  GsmClient*    sockets[8];
};

typedef TinyGsm::GsmClient TinyGsmClient;



//#endif
