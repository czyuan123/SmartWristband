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
#include <Adafruit_MLX90614.h>
Adafruit_MLX90614 mlx = Adafruit_MLX90614();

//SSL and Azure library
#include <ArduinoBearSSL.h>
#include<ArduinoECCX08.h>
#include <utility/ECCX08SelfSignedCert.h>
#include <ArduinoMqttClient.h>

// sensors includes
#define heartratePin A3
#include "DFRobot_Heartrate.h"
DFRobot_Heartrate heartrate(DIGITAL_MODE);
const int button = 4;

// time Includes
#include "ArduinoLowPower.h"

// SSL globals
GSMClient     gsmClient;            // Used for the TCP socket connection
BearSSLClient sslClient(gsmClient); // Used for SSL(secure socket layer)/TLS connection, integrates with ECC508
MqttClient    mqttClient(sslClient);

// iot hub globals
const char broker[] = SECRET_BROKER;
String deviceId = SECRET_DEVICEID;
// PIN Number
const char PINNUMBER[] = "";

char payload[] = "{ \"%s\": \"%s\", \"%s\": %f, \"%s\": %f, \"%s\": %d, \"%s\": %d, \"%s\": %f, \"%s\": \"%s\", \"%s\": %f }"; // deviceID, geoLat, geoLng, isWorn, heartrate, temperature, time

// time globals
unsigned int localPort = 2390;      // local port to listen for UDP packets
IPAddress timeServer(129, 6, 15, 28); // time.nist.gov NTP server address

const int NTP_PACKET_SIZE = 48; // NTP time stamp is in the first 48 bytes of the message
byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming and outgoing packets
GSMUDP Udp; // UDP instance to send and receive packets over UDP
// GSM globals
GSMClient client;
GPRS gprs;
GSM gsmAccess;
GSMVoiceCall vcs;
RTCZero rtc;

// Location globals
GSMLocation location;

// functions that are executed in other files
unsigned long epoch;
bool isCalled = false;

unsigned long returnTime()
{
  return epoch;
}

void MsgReceived(int msgSize)
{
  Serial.println("[loop/ Iothub / MsgReceived]");

   //print out content of the msg received
    Serial.print("[loop] Received a message with topic '");
    Serial.print(mqttClient.messageTopic());
    Serial.print("', length ");
    Serial.print(msgSize);
    Serial.println(" bytes:");

   //Use stream interface to print the contents
    while(mqttClient.available())
    {
      Serial.print((char)mqttClient.read());
    }
      Serial.println();
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

   float getTemp()  //get temperature of the patient
   {
    Wire.setClock(100000);
    Serial.println("[loop/ getInfo / getTemp]");
    
    float tempVal = 36.5 - 33.0;                //get offset value for calibration
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
      //Serial.println(HrateVal);
    }

    return 0;
  }

  int processHr() // process the heart rate
  {
    Serial.println("[loop / getInfo / processHr]");

    Serial.println("[loop] Getting Heart Rate");
    int hr = 0;

    Serial.println("[loop] Ensure device is worn accordingly");

    for (int k = 0; k < 30; k++)
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

struct Iothub
{
  void prepareConnection() // establish connection to hub after wake up
  {
    Serial.println("[loop / Iothub / prepareConnection]");

    while (!mqttClient.connected())
    {
      connectMQTT();
      mqttClient.poll();
    }
  }

  void connectMQTT() // connect to IoT Hub
  {
    Serial.println("[loop / Iothub / connectMQTT]");
    Serial.print("[loop] Attempting to connect to MQTT broker ");
    Serial.print(broker);
    if(isCalled == true){
      makecall();
    }
    while (!mqttClient.connect(broker, 8883))
    {
      // failed, retry
      Serial.print(".");
      Serial.println(mqttClient.connectError());
      //delay(1000);
    }
    Serial.println();

    Serial.println("[loop] Connected to the MQTT broker");
    Serial.println("");
    Serial.println("");
    Serial.println("");

    // subscribe to a topic
    mqttClient.subscribe("devices/" + deviceId + "/messages/devicebound/#");
    
  }

  void publishMessage(String messageToSend) // loop that sends message
  {
    led.onled();
    Serial.println("[loop / Iothub / publishMessage]");
    Serial.println("[loop] Sending message: " + String(messageToSend));

    // send message, the Print interface can be used to set the message contents
    mqttClient.beginMessage("devices/" + deviceId + "/messages/events/");
    mqttClient.print(messageToSend);
    mqttClient.endMessage();

    Serial.println("[loop] Message Sent");
    Serial.println();
    led.offled();
  }

  String compileMessage(char deviceID[], float geoLat, float geoLng, int Worn, int heartrate, float temperature, char t[], float battery) // compiles message of type 2 (telemetry)
  {
    Serial.println("[loop / Iothub / compileMessage]");
    // compile the data into the json payload
    char buffer[200];
    sprintf(buffer, payload, "id", deviceID, "geoLat", geoLat, "geoLng", geoLng, "isWorn", Worn, "heartrate", heartrate, "temperature", temperature, "time", t, "battery", battery);

    // return
    return String(buffer);
  }
};

Iothub iot;

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
    iothub(); // prepare connection to the IoT Hub
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


  void iothub() // connect to IoT Hub and prepare certificates
  {
    Serial.println("[loop / Setup / iothub]");
    Serial.println("[setup] Setting up Security Chip");
   
    if (!ECCX08.begin())
    {
      Serial.println("[setup] ERROR: No ECCX08 present!");
      while (1);
    }
   
    // reconstruct the self signed cert
    ECCX08SelfSignedCert.beginReconstruction(0, 8);
    ECCX08SelfSignedCert.setCommonName(ECCX08.serialNumber());
    ECCX08SelfSignedCert.endReconstruction();

    // Set a callback to get the current time
    // used to validate the servers certificate
    Serial.println("[setup] Signing certificates on time");
    time.processTime();
    ArduinoBearSSL.onGetTime(returnTime);
    delay(1000);

    // Set the ECCX08 slot to use for the private key
    // and the accompanying public certificate for it
    sslClient.setEccSlot(0, ECCX08SelfSignedCert.bytes(), ECCX08SelfSignedCert.length());

    Serial.println("[setup] Setting up variables");
    // Set the client id used for MQTT as the device id
    mqttClient.setId(deviceId);

    // Set the username to "<broker>/<device id>/api-version=2018-06-30" and empty password
    String username;

    username += broker;
    username += "/";
    username += deviceId;
    username += "/?api-version=2018-06-30";

    mqttClient.setUsernamePassword(username, "");

    // Set the message callback, this function is
    // called when the MQTTClient receives a message
    Serial.println("[setup] Preparing callbacks");
    mqttClient.onMessage(MsgReceived);
    Serial.println("");
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
    if(mqttClient.connected()){
    get.getTemp(); // get the temperature
    }
    get.batteryLevel(); // get the battery level
    
    if (get.isOn) // only get the heartrate if the device is worn
    {
      get.processHr(); // get the heartrate
    }
    else
    {
      get.tempV = 0;
    }

    // package and send data to cloud
    iot.publishMessage(iot.compileMessage(DEVICE_ID, loc.latitude, loc.longitude, get.isOn, get.heartrateV, get.tempV, time.timestamp, get.battery));

    // reset variables
    get.tempV = 0;
    get.heartrateV = 0;
    return 1;
  }

  // runs when the device wakes up
  int prepareWakeUp() // runs when the device wakes up
  {
    Serial.println("[loop / Looping / prepareWakeUp]");
    iot.connectMQTT();
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
  LowPower.attachInterruptWakeup(button,readbutton,HIGH);
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
 
  while(looping.prepareWakeUp()){
     if(isCalled == true){
  makecall();
 }
  }
  
  Serial.println("[loop] Sleeping for " + String(looping.sleepTime) + "ms");
  Serial.println("");
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
