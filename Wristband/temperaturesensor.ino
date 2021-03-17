#include <Adafruit_MLX90614.h>

//Code for MLX90614 IR temperature sensor
//Range for ambient temperature: -40 to 125 ˚C (-40 to 257 °F)
//Range for object temperature (non contact): -70 to 380 ˚C (-94 to 716 °F)
//Resolution: 0.02 °C
//Accuracy: 0.5°C for (0-50 °C) both ambient and object
#include <Wire.h>

#define SDA A4
#define SCL A5

Adafruit_MLX90614 mlx = Adafruit_MLX90614();

void setup() {
  Serial.begin (9600);
  mlx.begin();
  //Wire.begin(SDA,SCL);

}

void loop() {
 float temp = 36.5 - 34;
 float tempVal = temp + mlx.readObjectTempC();
 
 Serial.print("Ambient =  ");
 Serial.print(mlx.readAmbientTempC());
 Serial.println(" C");
 
 
 Serial.print("Target =  ");
 Serial.print(tempVal);
 Serial.println(" C");

 delay(1000);
  Wire.endTransmission();
}
