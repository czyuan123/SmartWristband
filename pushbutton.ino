#include <MKRGSM.h>

// Please enter your sensitive data in the Secret tab or arduino_secrets.h
// PIN Number
const char PINNUMBER[] = "";

// initialize the library instance
GSM gsmAccess; // include a 'true' parameter for debug enabled
GSMVoiceCall vcs;

String remoteNumber = "+60124128696";  // the number you will call
char charbuffer[20];

//Emergency button code
const int button = 4; //button pin number

void makecall(){

      // make sure the phone number is not too long:
      if (remoteNumber.length() < 20) {
        // let the user know you're calling:
        Serial.print("Calling to : ");
        Serial.println(remoteNumber);
        Serial.println();

        // Call the remote number by converting the number to char array
        remoteNumber.toCharArray(charbuffer, 20);


        // Check if the receiving end has picked up the call
        if (vcs.voiceCall(charbuffer)) {
          Serial.println("Call Established. Enter line to end");
          // Wait for some input from the line
          //to disconnect from call status of '1' means connected 
          while (Serial.read() != '\n' && (vcs.getvoiceCallStatus() == TALKING));
          // And hang up
          vcs.hangCall();
        }
        Serial.println("Call Finished");
      } 
    } 
 
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(button, INPUT);// button as input pin

  // connection state
  bool connected = false;

  // Start GSM shield
  // If your SIM has PIN, pass it as a parameter of begin() in quotes
  while (!connected) {
    //connect to network by calling gsmAccess.begin()
    if (gsmAccess.begin() == GSM_READY) {
      connected = true;
    } else {
      Serial.println("Not connected");
      delay(1000);
    }
  }
  Serial.println("Make Voice Call");
  Serial.println("GSM initialized.");
}

bool test(){
  Serial.println("hi");
  return 1;
}
int Bstate = 0;
int laststate = 0;

  
void readbutton(){
   Bstate = digitalRead(button);
  //check if button released
  //led light up if button released
  if (Bstate != laststate ){
  if (Bstate == HIGH){
    Serial.println("pressed");
    //makecall();
  }
 }
  laststate = Bstate;
  delay(500);
}
void loop() {
  //initial button state
  // put your main code here, to run repeatedly:
 
  while(test()){
    readbutton();
  }
  delay(100);
}
