#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <AltSoftSerial.h>
//#include <ArduinoJson.h>
#define RESET_PIN 2
#define HORN_PIN 5
#define CONTACT_PIN 6 
#define IGNITION_PIN 12
#define VIBRATION_PIN 10

SoftwareSerial sim800l(4,3);
AltSoftSerial neogps;
TinyGPSPlus gps;
//StaticJsonBuffer<200> jsonBuffer;

unsigned long previousMillis = 0;
unsigned long lastVibTime = 0;
unsigned long activationTime = 0;
bool alertFlag = false;
bool alarmFlag = false;
bool alarm = true;
long interval = 25000;
long alertTime = 10000;
int alarmTime = 15000;
String CellNumtemp;
String CellNum;
String Data_SMS;
short FIND_OK = -1;
short RING_OK = -1;
short OFF_OK = -1;
short ON_OK = -1;
short ALARM_OFF_OK = -1;
short ALARM_ON_OK = -1;
String Name = "Anarkita";

boolean getResponse(String expected_result, unsigned int timeout) { 
  boolean flag = false;
  String response = "";
  unsigned long previous;
  for (previous = millis(); (millis() - previous) < timeout;) {
    while (sim800l.available()) {
      response = sim800l.readString();
      if (response.indexOf(expected_result) > 0) {
        flag = true;
        goto OUTSIDE;
      }
    }
  }
  OUTSIDE:
    if (response != "") {
      Serial.println(response);
    }
    return flag;
}

boolean tryATcommand(String cmd, String expected_result, int timeout, int total_tries, bool reset) {
  TryAgain:
    for (int i = 1; i <= total_tries; i++) {
      sim800l.println(cmd);
      if (getResponse(expected_result, timeout) == true) {
        break;
      }
      else {
        Serial.println(".");
      }
      if (i == total_tries && reset) {
        Serial.println("Reset.");
        digitalWrite(RESET_PIN, LOW);
        delay(100);
        digitalWrite(RESET_PIN, HIGH);
        Init();
        goto TryAgain;
      } else if (i == total_tries) {
        Init();
        break;
      }
    }
}

void setup() {
  pinMode(RESET_PIN, OUTPUT);
  pinMode(HORN_PIN, OUTPUT);
  pinMode(CONTACT_PIN, OUTPUT);
  
  //DynamicJsonBuffer jsonBuffer;
  
  Serial.begin(9600);
  sim800l.begin(9600);
  neogps.begin(9600);
  
  digitalWrite(RESET_PIN, LOW);
  delay(100);
  digitalWrite(RESET_PIN, HIGH);
  digitalWrite(HORN_PIN, HIGH);
  digitalWrite(CONTACT_PIN, HIGH);
  
  Init();
}

void loop() {

  // Serial.println(digitalRead(VIBRATION_PIN));
  
  while(Serial.available()){
   sim800l.write(Serial.read());
  }
  
  vib();
  
  while (sim800l.available()) {
    Serial.write(sim800l.read());
    String dato = sim800l.readString();
    CellNumtemp = dato.substring(dato.indexOf("+56"));
    CellNum = CellNumtemp.substring(0, 12);
    Serial.println(dato);
    Serial.println(CellNum);
    FIND_OK = dato.indexOf(Name + " find");
    RING_OK = dato.indexOf("RING");
    OFF_OK = dato.indexOf(Name + " off");
    ON_OK = dato.indexOf(Name + " on");
    ALARM_OFF_OK = dato.indexOf(Name + " alarm off");
    ALARM_ON_OK = dato.indexOf(Name + " alarm on");
  }
  
  if (FIND_OK != -1) {
    find();
    FIND_OK = -1;
  }
  
  if (RING_OK != -1) {
    tryATcommand("ATH", "OK", 2000, 20, true);
    RING_OK = -1;
  }

  if (OFF_OK != -1) {
    digitalWrite(CONTACT_PIN, LOW);
    find();
    OFF_OK = -1;
  }

  if (ON_OK != -1) {
    digitalWrite(CONTACT_PIN, HIGH);
    find();
    ON_OK = -1;
  }

  if (ALARM_OFF_OK != -1) {
    alarm = false;
    alarm_sound(2);
    ALARM_OFF_OK = -1;
  }

  if (ALARM_ON_OK != -1) {
    alarm = true;
    alarm_sound(1);
    ALARM_ON_OK = -1;
  }
  
  if (millis() - previousMillis > 15000) {
    //sendGpsToServer();
    previousMillis = millis();
  }
  
}

void sendGpsToServer() {
  
  boolean newData = false;
  for (unsigned long start = millis(); millis() - start < 2000;) {
    while (neogps.available()) {
      if (gps.encode(neogps.read())) {
        newData = true;
        break;
      }
    }
  }
  //If newData is true
  if (true) {
    newData = false;
    String latitude, longitude;
    float altitude;
    unsigned long date, time, speed, satellites;

    latitude = String(gps.location.lat(), 6); // Latitude in degrees (double)
    longitude = String(gps.location.lng(), 6); // Longitude in degrees (double)
    altitude = gps.altitude.meters(); // Altitude in meters (double)
    date = gps.date.value(); // Raw date in DDMMYY format (u32)
    time = gps.time.value(); // Raw time in HHMMSSCC format (u32)
    speed = gps.speed.kmph();
    //ignition = digitalRead(IGNITION_PIN);

    String url, temp;
    url = "http://r15tracker.000webhostapp.com/gpsdata.php?lat=222&lng=222";

    tryATcommand("AT+CFUN=1", "OK", 2000, 20, true);
    tryATcommand("AT+CGATT=1", "OK", 2000, 3, false);
    tryATcommand("AT+SAPBR=3,1,\"Contype\",\"GPRS\"", "OK", 2000, 3, false);
    tryATcommand("AT+SAPBR=3,1,\"APN\",\"internet\"", "OK", 2000, 3, false);
    tryATcommand("AT+SAPBR=1,1", "OK", 2000, 3, false);
    tryATcommand("AT+HTTPINIT", "OK", 2000, 3, false);
    tryATcommand("AT+HTTPPARA=\"CID\",1", "OK", 1000, 3, false);
    sim800l.print("AT+HTTPPARA=\"URL\",\"");
    sim800l.print(url);
    tryATcommand("\"", "OK", 1000, 3, false);
    tryATcommand("AT+HTTPACTION=0", "0,200", 1000, 20, false);
    tryATcommand("AT+HTTPTERM", "OK", 1000, 3, false);
    tryATcommand("AT+CIPSHUT", "SHUT OK", 1000, 3, false);
    tryATcommand("AT+SAPBR=0,1", "OK", 2000, 3, false);
  }
  
}

void find() { 
  if(digitalRead(CONTACT_PIN) == HIGH){
    Data_SMS = Name + " is on";
  } else {
    Data_SMS = Name + " is off";
  }
  
  boolean newData = false;
  for (unsigned long start = millis(); millis() - start < 2000;) {
    while (neogps.available()) {
      if (gps.encode(neogps.read())) {
        newData = true;
        break;
      }
    }
  }
  
  //If newData is true
  if (true) {
    newData = false;
    Data_SMS = Data_SMS + "\nwww.google.com/maps/place/" + String(gps.location.lat(), 6) + "," + String(gps.location.lng(), 6);
  }
  
  Send_SMS();
}

void Send_SMS(){

  cancel_alarm();
  
  Serial.println("Sending sms...");
  Serial.println(Data_SMS);
  
  TryAgain:
    // tryATcommand("AT+CMGF=1", "OK", 1000, 3, false);
    String command = "AT+CMGS=\"" + CellNum + "\"";
    tryATcommand(command, CellNum, 1000, 3, false);
    delay(500);
    sim800l.print(Data_SMS);
    delay(500);
    sim800l.write(26);  // Required to tell the module that it can send the SMS
    for (unsigned long previous = millis(); (millis() - previous) < 20000;) {
      while (sim800l.available()) {
        String response = sim800l.readString();
        if (response.indexOf("ERROR") > 0) {
          goto TryAgain;
        }
      }
    }
    delay(500);
    sim800l.println();
    Serial.println("Data Sent.");
    delay(500);

}

void vib() {
  alarm_loop();
  if(alarm == true){
    if (digitalRead(IGNITION_PIN) == LOW) {
      if (digitalRead(VIBRATION_PIN) == HIGH) {
        if(!alertFlag){
          lastVibTime = millis();
          Serial.println("Alert ON");
          alarm_sound(1);
          alertFlag = true;
        } else {
          if (
            (millis() - lastVibTime) > alertTime && 
            (millis() - lastVibTime) < alertTime + alarmTime &&
            !alarmFlag
          ){
            Serial.println("Alarm ON");
            activationTime = millis();
            digitalWrite(HORN_PIN, LOW);
            alarmFlag = true;
          }
        }
      } else {
        if((millis() - lastVibTime) > alertTime + alarmTime){
          if(alarmFlag && (millis() - lastVibTime) > alertTime + alarmTime + 10000){
            Serial.println("Alarm OFF");
            digitalWrite(HORN_PIN, HIGH);
            alarmFlag = false;
          }
          if(alertFlag){
            Serial.println("Alert OFF");
            alertFlag = false; 
          }
        }
      }
    } else {
      cancel_alarm();
    } 
  } else {
    cancel_alarm();
  }
}

void alarm_loop(){
  if(alarmFlag){
    static long blinker = activationTime; 
    if((millis() - blinker) > 500){
      blinker = millis();
      digitalWrite(HORN_PIN, !digitalRead(HORN_PIN));
    } 
  } 
}

int alarm_sound(int times){
  for(int i = 0; i < times; i++){
    digitalWrite(HORN_PIN, LOW);
    delay(150);
    digitalWrite(HORN_PIN, HIGH);
    delay(150);
  }
}

void cancel_alarm(){
  if (alertFlag || alarmFlag) {
    digitalWrite(HORN_PIN, HIGH);
    Serial.println("Alert TOFF");
    Serial.println("Alarm TOFF");
    alertFlag = false;
    alarmFlag = false;
  }
}

void Init() {
  Serial.println("Init");
  tryATcommand("AT", "OK", 1000, 20, true);
  tryATcommand("AT+CMGF=1", "OK", 1000, 20, true);
  tryATcommand("AT+CNMI=2,2,0,0,0", "OK", 1000, 20, true);
}
