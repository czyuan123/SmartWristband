/*Debug LED
  On: Long Process
  Slow blink: Data Transfer (device to cloud)
  Fast blink: Data collection (sensors to device)
*/

//Include library
#include <MKRGSM.h>
#include <RTCZero.h>
#include "secrets.h"
//IR temperature library
#include <Wire.h>
#include <avr/dtostrf.h>
#include <Adafruit_MLX90614.h>
Adafruit_MLX90614 mlx = Adafruit_MLX90614();

#include <ArduinoMqttClient.h>
#include <PubSubClient.h>

// sensors includes
#define heartratePin A3
#include "DFRobot_Heartrate.h"
DFRobot_Heartrate heartrate(DIGITAL_MODE);
const int button = 4;

// time Includes
#include "ArduinoLowPower.h"

//Settings for the MQTT client
//IP address of the server (set in arduino_secrets.h)
//const char deviceID[] = DEVICE_ID; 
const char username[] = USER;
const char passwd[]= PWD;
const char server[]   = MQTT_BROKER;
const char inTopic[]  = "wristband/serverToClient";
const char outTopic[] = "wristband/client";
const char clientID[] = "SmartCLI";

// PIN Number
const char PINNUMBER[] = "";

//Variable to store the MQTT message/payload
char pubCharsPayload[500];

// time globals
unsigned int localPort = 2390;      // local port to listen for UDP packets
IPAddress timeServer(129, 6, 15, 28); // time.nist.gov NTP server address

const int NTP_PACKET_SIZE = 48; // NTP time stamp is in the first 48 bytes of the message
byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming and outgoing packets
GSMUDP Udp; // UDP instance to send and receive packets over UDP
// GSM globals
GSMClient gsmClient;
GPRS gprs;
GSM gsmAccess;
GSMVoiceCall vcs;
RTCZero rtc;

// Location globals
GSMLocation location;

void callback(char* topic, byte* payload, unsigned int length) {
  //Handle received messages
}

PubSubClient mqttClient(server,1883,callback,gsmClient);

// functions that are executed in other files
unsigned long epoch;
bool isCalled = false;

unsigned long returnTime()
{
  return epoch;
}

void alarmEvent0(){
    Serial.begin(9600);
}

//debug LED
struct LED
{
    int ledPin = LED_BUILTIN;

    void onled()
    {
      digitalWrite(ledPin,HIGH);
    }

    void offled()
    {
      digitalWrite(ledPin,LOW);
    }

    void blink(int wait)
    {
      digitalWrite(ledPin,HIGH);  //on
      delay(wait);
      digitalWrite(ledPin,LOW);   //off
      delay(wait);
    }
};

LED led;

struct getInfo
{
   bool isOn;
   float tempV;
   float battery;
   int heartrateV;
   int k;
   String Stats;

   float getTemp()  //get temperature of the patient
   {
    Wire.setClock(100000);
    Serial.println("[loop/ getInfo / getTemp]");
    
    float tempVal = 36.5 - 34.0;                //get offset value for calibration
    float temp = tempVal + mlx.readObjectTempC();//add offset to every reading
    tempV = temp;
    Serial.print("Target = ");
    Serial.print(String(temp));
    Serial.println(" C");

    return temp;
   
  }

  int getHr() // query the heart rate
  {
    uint8_t HrateVal;
    heartrate.getValue(heartratePin);
    HrateVal = heartrate.getRate();
    k++;

    if (HrateVal)
    {
      return HrateVal;
    }

    return 0;
  }

  int processHr() // process the heart rate
  {
    Serial.println("[loop / getInfo / processHr]");

    Serial.println("[loop] Getting Heart Rate");
    int hr = 0;

    Serial.println("[loop] Ensure device is worn accordingly");

    for (int k = 0; k < 50; k++)
    {
      hr = getHr();
      led.blink(100);

      if (hr > 0)
      {
        break;
      }
    }

    Serial.println("[loop] Heart rate is " + String(hr));
    heartrateV = hr;
    return hr;
  }

bool isWorn() // checks if the user is wearing the device.
  {
    Serial.println("[loop / getInfo / isWorn]");

    Serial.println("[loop] Checking if the device is being worn");
    int pozCount = 0; // counts the number of positive reads
    int sampleSpace = 50;
    int margin = 5;

    // should work in most instances. Note that if organic material is in the range of the hr sensor, it will think someone is wearing it
    for (int k = 0; k < sampleSpace; k++)
    {
      int val = heartrate.getValue(heartratePin);
      Serial.print(".");

      if (val > 1000) // read confirms that device is not worn
      {
        pozCount++;
      }
      else
      {
        pozCount = 0;
      }

      led.blink(100);
    }

    Serial.println("");

    if (pozCount > margin)
    {
      Serial.println("[loop] Device is not worn");
      isOn = false;
      return false;
    }

    Serial.println("[loop] Device is worn");
    isOn = true;
    return true;
  }

  void batteryLevel()
   {
    Serial.println("[loop / getInfo / batteryLevel]");
    int sensorValue = analogRead(ADC_BATTERY);
    float voltage = sensorValue * (4.3 / 1023.0);
    battery = voltage;
  }

  void statuslevel(){
    if(tempV > 38 || 50 < heartrateV < 110){
      Stats = "BAD";
    }
    Stats = "GOOD";
  }
};

getInfo get;

struct Time
{
  int timeZone = 8; // difference from GMT
  char timestamp[25]; // stores the time in the according format (YYYY-MM-DDThh:mm:ssZ)

  unsigned long getEpochTime() // gets the time from the server
  {
    Serial.println("[loop / Time / getEpochTime]");
    Serial.println("[setup] asking server for time");
    sendNTPpacket(timeServer); // send an NTP packet to a time server
    delay(1000); // delay is essential, do not delete

    if ( Udp.parsePacket() )
    {
      Serial.println("[setup] packet received");
      Udp.read(packetBuffer, NTP_PACKET_SIZE); // read the packet into the buffer

      unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
      unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);

      // combine the four bytes (two words) into a long integer
      // this is NTP time (seconds since Jan 1 1900):
      unsigned long secsSince1900 = highWord << 16 | lowWord;

      // now convert NTP time into everyday time
      // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
      const unsigned long seventyYears = 2208988800UL;
      const unsigned long timeZoneDiff = timeZone * 3600; // adjust the time zone

      // subtract seventy years and add time zone
      unsigned long epoch = (secsSince1900 - seventyYears) + timeZoneDiff;

      Serial.println("[setup] Packet Read");
      Serial.println("[setup] Server epoch time (local)  " + String(epoch));
      return epoch;
    }
    else
    {
      return 0;
    }
  }
  void synchRTC(unsigned long epoch) // synchs the onboard RTC to that time
  {
    Serial.println("[loop / Time / synchRTC]");
    Serial.println("[setup] Initialising RTC with Time");
    Serial.println("[setup] Initialising RTC");
    rtc.begin();

    Serial.println("[setup] Setting Current Time");
    rtc.setEpoch(epoch);

    Serial.println("[setup] RTC Setup Complete");
  }

  String processTime() // a public loop to convert time to string
  {
    Serial.println("[loop / Time / processTime]");
    String time;
    String date;
    String finalVal;

    date += "20"; // the RTC library can't return the full year, have to add 20 to start to get 20xx
    date += rtc.getYear();
    date += "-";
    date += rtc.getMonth();
    date += "-";
    date += rtc.getDay();

    if (rtc.getHours() < 10) time += "0";
    time += rtc.getHours();
    time += ":";
    if (rtc.getMinutes() < 10) time += "0";
    time += rtc.getMinutes();
    time += ":";
    if (rtc.getSeconds() < 10) time += "0";
    time += rtc.getSeconds();

    rtc.getY2kEpoch();
    epoch = rtc.getEpoch();

    finalVal = date + String("T") + time + String(".0Z");
    finalVal.toCharArray(timestamp, finalVal.length());
    Serial.println("[loop] Time is " + String(timestamp));
    return finalVal;
  }

  unsigned long sendNTPpacket(IPAddress& address)
  {
    memset(packetBuffer, 0, NTP_PACKET_SIZE);
    packetBuffer[0] = 0b11100011;   // LI, Version, Mode
    packetBuffer[1] = 0;     // Stratum, or type of clock
    packetBuffer[2] = 6;     // Polling Interval
    packetBuffer[3] = 0xEC;  // Peer Clock Precision
    packetBuffer[12]  = 49;
    packetBuffer[13]  = 0x4E;
    packetBuffer[14]  = 49;
    packetBuffer[15]  = 52;

    Udp.beginPacket(address, 123); //NTP requests are to port 123
    Udp.write(packetBuffer, NTP_PACKET_SIZE);
    Udp.endPacket();
  }
};

Time time;

struct Location
{
  float latitude;
  float longitude;

  void getLocation()
  {
    Serial.println("[loop / Location / getLocation]");
    lockLocation();
  }

  int lockLocation() // queries the location until it locks
  {
    Serial.println("[loop / Location / lockLocation]");
    Serial.println("[loop] Attempting to lock");

    while (true)
    {
      led.blink(200);
      Serial.print(".");
      if (location.available())
      {
        if(location.accuracy() < 200000){
        latitude = location.latitude();
        longitude = location.longitude();

        if (checkLocation(latitude, longitude))
        {
          break;
        }
        }
      }
    }

    Serial.println("");
    Serial.println("[loop] Location is " + String(latitude) + ", " + String(longitude));
    return 1;
  }

  int checkLocation(float lat, float lng) // checks if the location locked
  {
    int rawLat = lat;
    int rawLng = lng;

    if (rawLat == lat && rawLng == lng) // location is not precise
    {
      return 0;
    }
    else
    {
      return 1;
    }
  }
};

Location loc;

struct Nodejs
{
 
  void publishMessage(){
    Serial.println("[loop / Nodejs / prepareConnection]");
    if(mqttClient.connect(clientID)){
    
    char bufferBat[20];
    char bufferTemp[20];
    char bufferLat[20];
    char bufferLng [20];
    char bufferHR[20];
    String lat = dtostrf(loc.latitude,10,7,bufferLat);
    String lng = dtostrf(loc.longitude,10,7,bufferLng);
    String temperature = dtostrf(get.tempV,10,7,bufferTemp);
    String heartrate = itoa(get.heartrateV, bufferHR,10);
    String battery = dtostrf(get.battery,10,7,bufferBat);
    String t = String(time.timestamp);
    char Name[] = "John";
    String id = String(Name);
    
    String payload = "{\"Latitude\":"+lat+",\"Longitude\":"+lng+",\"Temperature\":"+temperature+",\"HeartRate\":"+heartrate+",\"Battery\":"+battery+",\"isWorn\":"+get.isOn+",\"Status\":"+get.Stats+"}";
    payload.toCharArray(pubCharsPayload, (payload.length() + 1));
    mqttClient.publish(outTopic, pubCharsPayload);
    
    Serial.println("[loop] Message Sent");
    Serial.println("MQTT Published");
    Serial.println(pubCharsPayload);}
     else {
    Serial.println("MQTT client can't connect!");
    mqttReconnect();
  }
  mqttClient.loop();
  delay(1000);
  }

  void mqttReconnect() {
  //Loop until we're reconnected
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    //Attempt to connect
    if (mqttClient.connect(clientID)) {
      Serial.println("MQTT client connected");
      //Once connected, publish an announcement...
      mqttClient.publish(outTopic, clientID);
      //... and resubscribe
      mqttClient.subscribe(inTopic);
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      //Wait 5 seconds before retrying
      delay(2000);
      }
    }
  }
};

Nodejs node;

struct Setup
{
  void run() // set up the device
  {
    LowPower.attachInterruptWakeup(RTC_ALARM_WAKEUP, alarmEvent0, CHANGE);

    Serial.println("[loop / Setup / run]");
    gsm(); // prepare GSM

    Serial.println("");
    rtc(); // preapre time and RTC

    Serial.println("");
    ledSet(); // set up the onboard LED

    location.begin(); // set up the lcoation
    loc.getLocation(); // lock onto the location
  }
  
  void gsm() // set up GSM
  {
    Serial.println("[loop / Setup / gsm]");
    Serial.println("[setup] Starting Arduino GPRS NTP client");

    // connection state
    bool connected = false;

    while (!connected)
    {
      if ((gsmAccess.begin() == GSM_READY) && (gprs.attachGPRS("", "", "") == GPRS_READY))
      {
        connected = true;
      }
      else
      {
        Serial.print(".");
        led.blink(500);
      }
    }
    Serial.println("GSM initialized.");
}
  
  void ledSet() // set up the onboard LED
  {
    Serial.println("[loop / Setup / led]");
    pinMode(led.ledPin, OUTPUT);
    led.offled();
  }

  void rtc() // get the time and synch it to the onboard RTC
  {
    Serial.println("[loop / Setup / rtc]");
    Serial.println("[setup] Starting connection to server");
    Udp.begin(localPort);

    Serial.println("[setup] Synching onboard RTC to GNSS Time");

    // wait until we have the time appended to a variable
    unsigned long epoch = 0;
    while (!epoch)
    {
      Serial.println(".");
      epoch = time.getEpochTime();
    }

    Serial.println("");

    // synch the time to the RTC
    time.synchRTC(epoch);
  }
};

Setup setting;

struct Looping
{
  int sleepTime = MODE; // the time to wait between reads

  int sendData()
  {
    Serial.println("[loop / Looping / sendTwo]");
    loc.getLocation(); // get the current location - wait until precise

    get.isWorn(); // check if the device is being worn
    //if(mqttClient.connected()){
    get.getTemp(); // get the temperature
    //}
    get.batteryLevel(); // get the battery level
    if (get.isOn) // only get the heartrate if the device is worn
    {
      get.processHr(); // get the heartrate
    }
    else
    {
      get.tempV = 0;
    }
    get.statuslevel();
      node.publishMessage();
    
    // reset variables
    get.tempV = 0;
    get.heartrateV = 0;
    return 1;
  }

  // runs when the device wakes up
  bool prepareWakeUp() // runs when the device wakes up
  {
    Serial.println("[loop / Looping / prepareWakeUp]");
    time.processTime();
    sendData();
    return 1;
  }
};

Looping looping;
 
void readbutton(){
  Serial.println("pressed");
  //check if button released
  isCalled = true;
  }

void makecall(){
    //variable to store the phone number to call
      String remoteNumber = "+60124128696";  // the number you will call
      char charbuffer[20];

      // make sure the phone number is not too long:
        // let the user know you're calling:
        Serial.print("Calling to : ");
        Serial.println(remoteNumber);
        Serial.println();
        // Call the remote number by converting the number to char array
        remoteNumber.toCharArray(charbuffer, 20);

        // Check if the receiving end has picked up the call
        if (vcs.voiceCall(charbuffer)) {
          // Wait for some input from the line
          //to disconnect from call status of '1' means connected 
          while (vcs.getvoiceCallStatus() == TALKING);
          // And hang up
          vcs.hangCall();
        }
        Serial.println("Call Finished");
        isCalled = false;
}  

void setup()
{
  Serial.begin(9600);
  pinMode(button, INPUT);// button as input pin
  // only start the Serial Monitor if in development mode
  //LowPower.attachInterruptWakeup(button,readbutton,HIGH);
  attachInterrupt(digitalPinToInterrupt(button),readbutton,HIGH);
  if (MODE == DEVELOP_TIME)
  {
    while (!Serial);
  }
  Serial.println("SmartWristband");
  Serial.println("");
  Serial.println("");

  setting.run(); // get everything set up
}

void loop()
{
  led.offled();
  //delay(500);
  led.blink(1000);

 //looping.prepareWakeUp();
  while(looping.prepareWakeUp()){
     if(isCalled == true){
  makecall();
 }
  }

  Serial.println("[loop] Sleeping for " + String(looping.sleepTime) + "ms");
  Serial.println("");
  Serial.println("");
  
 if (MODE == DEVELOP_TIME) // delay 10 seconds if debugging
  {
    led.onled();
  }
  else // sleep for 5 minutes if released
  {
    led.blink(2000);
   LowPower.sleep(looping.sleepTime);
  }
}
