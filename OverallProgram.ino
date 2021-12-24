#include <Wire.h>
#include <avr/wdt.h>
//#include <EEPROM.h>
#include "BlueDot_BME280.h"
#include "MAX30105.h"
#include "spo2_algorithm.h"

MAX30105 particleSensor;
#define MAX_BRIGHTNESS 255

BlueDot_BME280 bme1;                                     //Object for Sensor 1
BlueDot_BME280 bme2;                                     //Object for Sensor 2
int bme1Detected = 0;                                    //Checks if Sensor 1 is available
int bme2Detected = 0;                                    //Checks if Sensor 2 is available

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
//Arduino Uno doesn't have enough SRAM to store 100 samples of IR led data and red led data in 32-bit format
//To solve this problem, 16-bit MSB of the sampled data will be truncated. Samples become 16-bit data.
uint16_t irBuffer[100]; 
uint16_t redBuffer[100];  
#else
 
#endif




 /*the initializing code for the peltier
  */
int peltier = 3; //The N-Channel MOSFET is on digital pin 3
int power = 99; //Power level fro 0 to 99%
int peltier_level = map(power, 0, 99, 0, 255); //This is a value from 0 to 255 that actually controls the MOSFET

int userDesired = 62;
int PeltierTemp = 0;
int AmbientTemp = 0;
int newDesired = 0;
//int addr = 0;

int count = 0;
int32_t spo2_avg = 0; 
int32_t heartRate_avg = 0; 
int32_t bufferLength; 
int32_t spo2; 
int8_t validSPO2; 
int32_t heartRate; 
int8_t validHeartRate; 

byte pulseLED = 11; 
byte readLED = 13; 

void setup() {
  Serial.begin(115200);

  pinMode(peltier, OUTPUT);
  Serial.println(F("Basic Weather Station"));

  //*********************************************************************
  //This program is set for the I2C mode
    bme1.parameter.communication = 0;                    //I2C communication for Sensor 1 (bme1)
    bme2.parameter.communication = 0;                    //I2C communication for Sensor 2 (bme2)

  //*********************************************************************
  //Set the I2C address of your breakout board  
    bme1.parameter.I2CAddress = 0x77;                    //I2C Address for Sensor 1 (bme1)
      //black sensor, SDO not connected to ground = peltier
    bme2.parameter.I2CAddress = 0x76;                    //I2C Address for Sensor 2 (bme2)
      //blue sensor, grounded connection for SDO = ambient 
      
  // Set which mode to run the device on 
    //0b00 - sleep mode, no measurements 
    ///0b01 - 1 measurement then go to sleep
   //0b11 - default mode to run the sensor   
    bme1.parameter.sensorMode = 0b11;                    //Setup Sensor mode for Sensor 1
    bme2.parameter.sensorMode = 0b11;                    //Setup Sensor mode for Sensor 2 
 
  //Internal IIR Filter - suppress high frequency fluctuations 
      //high factor value = less noise, but also less responsive measurements 
      //0b100 is default value to start at 
    bme1.parameter.IIRfilter = 0b100;                   //IIR Filter for Sensor 1
    bme2.parameter.IIRfilter = 0b100;                   //IIR Filter for Sensor 2
  
  //oversampling factor for temperature
      //0b000:      factor 0 (Disable temperature measurement)
      //0b101:      factor 16 (default value)
    bme1.parameter.tempOversampling = 0b101;              //Temperature Oversampling for Sensor 1
    bme2.parameter.tempOversampling = 0b101;              //Temperature Oversampling for Sensor 2
    
 //Watchdog timer - restart when program takes >8sec. 
 // wdt_enable(WDTO_8S);                                 //Watchdog Timer counts for 8 seconds before starting the reset sequence
 
 //Troubleshooting
  if (bme1.init() != 0x60){    
    Serial.println(F("Ops! Peltier BME280 Sensor not found!"));
    bme1Detected = 0;
  }
  else{
    Serial.println(F("Peltier BME280 Sensor detected!"));
    bme1Detected = 1;
  }

  if (bme2.init() != 0x60){    
    Serial.println(F("Ops! Ambient BME280 Sensor not found!"));
    bme2Detected = 0;
  }
  else{
    Serial.println(F("Ambient BME280 Sensor detected!"));
    bme2Detected = 1;
  }




  pinMode(pulseLED, OUTPUT);
  pinMode(readLED, OUTPUT);

  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) 
  {
    Serial.println(F("MAX30102 was not found. Please check wiring/power."));
    while (1);
  }

  Serial.println(F("Attach sensor to finger securely. Enter any ker to begin sampling."));
  while (Serial.available() == 0) ; 
  Serial.read();

  byte ledBrightness = 60; //Options: 0=Off to 255=50mA
  byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  byte sampleRate = 100; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 8192; //Options: 2048, 4096, 8192, 16384

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings

 Serial.println();
 Serial.println();
}

    
  
 
//void setup ends here


 //Measurements start here
void loop() {
  
  
  for (byte i = 0 ; i < bufferLength ; i++)
  {
    while (particleSensor.available() == false) //do we have new data?
      particleSensor.check(); //Check the sensor for new data

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample(); //We're finished with this sample so move to next sample

    Serial.print(F("red="));
    Serial.print(redBuffer[i], DEC);
    Serial.print(F(", ir="));
    Serial.println(irBuffer[i], DEC);
  }

  //calculate heart rate and SpO2 after first 100 samples (first 4 seconds of samples)
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

  //Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every 1 second
  while (1)
  {

  char option;
   // userDesired = analogRead(0) / 4;
   // EEPROM.write(addr, userDesired);
/*
 * manual option for user to increase or decrease peltier
 * temp, should they desire to control it
  */
  if(Serial.available() > 0){
    option = Serial.read();
    userDesired = 72;
    if(option == 'q'){
      userDesired = 62;
    }
    else if(option == 'u'){
      userDesired++;
    }
    else if(option == 'd'){
      userDesired--;
    }
    
    peltier_level = map(power, 0, 99, 0, 255);
  }

  Serial.print("q=62F, u=increase, d=decrease ");
  Serial.print("Power=");
  Serial.print(power);
  Serial.print(" PLevel=");
  Serial.println(peltier_level);
  analogWrite(peltier, peltier_level); //Write this new value out to the port
  /* 
   *  bme1 is peltier temp (black sensor) 
   *  bme2 is ambient temp (blue sensor)
   */
  PeltierTemp = bme1.readTempF();
  AmbientTemp = bme2.readTempF();

  //feedback loop
  if(PeltierTemp > (userDesired + 2)){   
    if(power > 2){
       power = power - 2;
     }
    else{
     power = 20;
    }   
  }
  else if(PeltierTemp < (userDesired - 2)){   
     if(power < 97){
      power = power + 2;
     }
     else{
     power = 70;
     }   
  }
  
  newDesired = (userDesired - AmbientTemp) + userDesired;
  //exclude line below if testing for errors
  userDesired = newDesired;

  if(userDesired > 75){
    userDesired = 75;
  }
  else if(userDesired < 60){
    userDesired = 60;
  }
  wdt_reset();

  Serial.print(F("Duration in Seconds:  "));
  Serial.println(float(millis())/1000);

  if(bme1Detected){
    Serial.print(F("Pelt Temperature Sensor 1 [°F]:\t\t")); 
    Serial.println(bme1.readTempF()); 
   /* to test errors 
    *  Serial.println(userDesired);
    */
    Serial.println(F("****************************************"));    
  }
  else{
    Serial.print(F("Temperature Sensor 1 [°F]:\t\t"));
    Serial.println(F("Null")); 
    /*to test errors
    *Serial.println((userDesired)); 
    */
    Serial.println(F("****************************************"));   
  }

  if(bme2Detected){
    Serial.print(F("Temperature Sensor 2 [°F]:\t\t")); 
    Serial.println(bme2.readTempF());
  }
  else{
    Serial.print(F("Temperature Sensor 2 [°F]:\t\t")); 
    Serial.println(F("Null"));
  }
   
   Serial.println();
   Serial.println();


    bufferLength = 100; //buffer length of 100 stores 4 seconds of samples running at 25sps





    
    //dumping the first 25 sets of samples in the memory and shift the last 75 sets of samples to the top
    for (byte i = 25; i < 100; i++)
    {
      redBuffer[i - 25] = redBuffer[i];
      irBuffer[i - 25] = irBuffer[i];
    }

    //take 25 sets of samples before calculating the heart rate.
    for (byte i = 75; i < 100; i++)
    {

      while (particleSensor.available() == false) 
        particleSensor.check(); 

      digitalWrite(readLED, !digitalRead(readLED)); 

      redBuffer[i] = particleSensor.getRed();
      irBuffer[i] = particleSensor.getIR();
      particleSensor.nextSample(); 

      Serial.print(F("HR="));
      Serial.print(heartRate/2, DEC);
      
      Serial.print(F(", HRvalid="));
      Serial.print(validHeartRate, DEC);
      
      Serial.print(F(", SPO2="));
      Serial.print(spo2, DEC);
      

      Serial.print(F(", SPO2Valid="));
      Serial.println(validSPO2, DEC);

      count = count + 1;
      spo2_avg = spo2_avg + spo2;
      heartRate_avg = heartRate_avg + heartRate;
      
      if(count == 10){
        spo2_avg = spo2_avg/10;
        heartRate_avg = heartRate_avg/10;
        Serial.print(F("HeartRateAVG="));
        Serial.print(heartRate_avg, DEC);
        Serial.print(F(", SPO2AVG="));
        Serial.print(spo2_avg, DEC);
        Serial.print("\n");
        spo2_avg = 0;
        heartRate_avg = 0;
        count = 0;
      }
      
      
      delay(20);
    }
    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
  
}
   delay(1000);

   
   
}
