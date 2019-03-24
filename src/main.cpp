#include <Arduino.h>
#include <TinyGPS++.h>
#include "TinyGsmClient_Elescrow_32u4A7.h"

#define APN "wap.tim.it"
#define USER "WAPTIM"
#define PASS "WAPTIM"

#define DEBUGER // if you define more information on serial will be displayed
#define DEBUGER2 // many additional info on serial will be displayed - dengerous many RAM usage ! alala

#define RESTARPERIOD 200
#define HEARTBIT 60

bool ConnectGSM();
void init_restart_A9();
void sleep(uint16_t count);
bool readytosend = true; // sending controll
uint8_t reporter = 0; // count for reporting after period of not movment
uint8_t reporterInvalid = 0; // count for reporting after period of not movment
volatile uint8_t stendby = 0;  //for SLEEPER and INTERRPUT
uint8_t restart = 0; //restart count if hang
uint8_t count = 50;  //counting of looping for control battery
static uint16_t volt = 0; //voltage percent if you want to use uint8_t bat
static double LAST_LAT = 0, LAST_LON = 0; //previous position storage

bool firstTime = true;

int noDataCount = 0;
int noMovementTimes = 0;

#define ss Serial1
TinyGPSPlus gps;  //set GPS object
TinyGsm modem(ss);  //assigne serial and name for A7 to object  and for standard AT operations
TinyGsmClient Client(modem); //Create GPRS connection obiect for network operation

uint16_t getFreeSram() {

extern unsigned int __bss_end;
extern unsigned int __heap_start;
extern void *__brkval;
uint8_t newVariable;
  // heap is empty, use bss as start memory address
  if ((uint16_t)__brkval == 0)
    return (((uint16_t)&newVariable) - ((uint16_t)&__bss_end));
  // use heap end as the start of the memory address
  else
    return (((uint16_t)&newVariable) - ((uint16_t)__brkval));
};

bool ConnectGSM () {


#ifdef DEBUGER
    Serial.println(F("Connecting to GPRS"));
#endif
while (!modem.ping()){
    Serial.println(F("Waiting modem ready"));
}
  modem.gprsConnect(F(APN), F(USER), F(PASS));


#ifdef DEBUGER
  Serial.println(F("GPRS connected"));
#endif


  return true;
}

void displayInfo()
{
    uint16_t distancemToLAST = 0;
    String atLocation = modem.getLocation();

    Serial.print(F("LocationAT: "));
    if(atLocation.indexOf(",") > 0){//Valid
      reporter++;
      Serial.println(atLocation);
      int firstCommaIndex = atLocation.indexOf(',');

      String latStr = atLocation.substring(0, firstCommaIndex);
      latStr.trim();
      //Serial.println(latStr);
      double lat = latStr.toDouble();
      //Serial.println(atLocation.length());
      //Serial.println(lat);
      double lon = atLocation.substring(firstCommaIndex+1, atLocation.length()).toDouble();

      distancemToLAST = (uint32_t)TinyGPSPlus::distanceBetween(lat,lon,LAST_LAT,LAST_LON);

      Serial.print("**** Distance: ");
      Serial.println(distancemToLAST);

      LAST_LAT= lat;
      LAST_LON= lon;
      if(distancemToLAST > 15 || firstTime || reporter > HEARTBIT){
        firstTime = false;
        reporter = 0;
        modem.sendCoordinate("location="+atLocation+"&batt="+String(modem.getBattVoltagemv())+"&altitude="+String(modem.getExtBatteryVoltage()));
        //modem.sendCoordinateMQTT("{'_type':'location','alt':"+String(modem.getExtBatteryVoltage())+",'lon':"+String(lon,7)+",'t':'u','batt':"+modem.getBattVoltage()+",'tid':'ta','acc':100,'lat':"+String(lat,7)+"}");

      }else{
        #ifdef DEBUGER
          Serial.print("No Movement sleeping distance: ");
          Serial.println(distancemToLAST);
          noMovementTimes++;

          if (noMovementTimes >30){

            /*Serial.print("Going To sleep....");
            modem.stopGPS();
            delay(240000L);
            Serial.print("I'm back!");
            modem.startGPS();*/
            noMovementTimes = 0;
          }
        #endif

      }

    }else{
      Serial.println("INVALID POSITION");
      if(reporterInvalid > HEARTBIT){
        modem.sendCoordinate("batt="+String(modem.getBattVoltagemv())+"&altitude="+String(modem.getExtBatteryVoltage()));
        //modem.sendCoordinateMQTT("{\"_type\":\"location\",\"alt\":"+String(modem.getExtBatteryVoltage())+",\"lon\":0.0,\"t\":\"u',\"batt\":"+String(modem.getBattVoltage())+",\"tid\":\"ta\",\"acc\":100,\"lat\":0.0}");
        reporterInvalid = 0;
        }else{
            reporterInvalid++;
        }
    }
  Serial.println();

}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  ss.begin(115200);

  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(8,OUTPUT);
  digitalWrite(5, HIGH);
  digitalWrite(4, LOW);
  digitalWrite(8, HIGH);
  delay(3000);
  digitalWrite(8, LOW);

  Serial.println("by Raf");
  Serial.println();
  init_restart_A9 ();
}

void loop() {
  // put your main code here, to run repeatedly:
  bool newData = false;
  unsigned long chars;
  unsigned short sentences, failed;
  bool dataFlowing = false;

    displayInfo();

    if(modem.ping()){
        //Serial.println("ping OK");
    }else{
        Serial.println("ping KO");
        while(!modem.ping()){
            Serial.println("Trying to wakeup the modem");
        }
    }

    delay(1000L);
}

void init_restart_A9() {

   delay(10);
#ifdef DEBUGER
  Serial.println(F("A9G power ON/OFF!"));
#endif
    yield();
    if (ss.availableForWrite() >0) {
      Serial.begin(115200);
      yield();

      int networkTries =0;
      while(!modem.ping()){
          Serial.println("Wait modem to be ready...");
      }

      while(!modem.waitForNetwork()){
          Serial.println("Wait network to be ready...");
          delay(1000L);
          networkTries++;
          if (networkTries > 60){
            networkTries = 0;
            modem.restart();
            while(!modem.ping()){
                Serial.println("Wait modem to be ready...");
            }
          }
      }
      modem.stopGPS();
      delay(1000);
     if (ConnectGSM()) {
        modem.startGPS();
        delay(1000);
      }
#ifdef  DEBUGER
    Serial.println(F("GPS starting"));

#endif

    delay((uint32_t)A7_GPS_PERIOD *(uint32_t)1000);
    yield();

    }  else {
#ifdef  DEBUGER2
    Serial.println(F("No Serial1 !!! "));
#endif
    }

}
