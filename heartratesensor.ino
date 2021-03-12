//code for heart rate sensor
//digital mode
#define heartratePin A3
#include "DFRobot_Heartrate.h"
DFRobot_Heartrate heartrate(DIGITAL_MODE); ///< ANALOG_MODE or DIGITAL_MODE

int getHr() // query the heart rate
  {
    uint8_t HrateVal;
    int k;
    heartrate.getValue(heartratePin);
    HrateVal = heartrate.getRate();
    k++;

    if (HrateVal)
    {
      return HrateVal;
      Serial.println(HrateVal);
    }
    delay(100);
    return 0;
  }

  int processHr() // process the heart rate
  {
    Serial.println("[loop / getInfo / processHr]");

    Serial.println("[loop] Getting Heart Rate");
    int hr = 0;

    Serial.println("[loop] Ensure device is worn accordingly");

    for (int k = 0; k < 35; k++)
    {
      hr = getHr();
      //led.blink(100);

      if (hr > 0)
      {
        break;
      }
    }

    Serial.println("[loop] Heart rate is " + String(hr));
    int heartrateV = hr;
    return hr;
  }
  
  bool isWorn() // checks if the user is wearing the device.
  {
    bool isOn;
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
      //Serial.println(String(val));

      if (val > 1000) // read confirms that device is not worn
      {
        pozCount++;
      }
      else
      {
        pozCount = 0;
      }

      //led.blink(100);
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

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  processHr();
  isWorn();
    /*uint8_t HrateVal;
    heartrate.getValue(heartratePin);
    HrateVal = heartrate.getRate();
    //k++;
    if (HrateVal)
    {
      Serial.println(HrateVal);
    }
    delay(10);*/
  
}
