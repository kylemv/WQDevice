//Data logger Demonstration using Seeeduino Stalker v3.0. Logs Battery Voltage every 10 seconds to DATALOG.CSV file
//Use this demo code to implement your Datalogger functionality, add additional sensors.

//1.Solder P3 and P2 PCB jumper pads
//2.Compile and upload the sketch
//3.See if everything works fine using Serial Monitor.
//4.Remove all Serial port code, recompile the sketch and upload.
// This reduces power consumption during battery mode.

#include <avr/sleep.h> 
#include <avr/power.h>
#include <Wire.h>
#include <AESLib.h>
#include <Adafruit_FONA.h>
#include <OneWire.h>
#include <DS1337.h>
#include <SPI.h>
#include <SD.h>
#include <SoftwareSerial.h>
#include <Math.h>
#include <Time.h>

// FONA.....................................
#define FONA_RX 2
#define FONA_TX 3
#define FONA_RST 4
char replybuffer[255]; // this is a large buffer for replies
#include <SoftwareSerial.h>
SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
SoftwareSerial *fonaSerial = &fonaSS;
Adafruit_FONA_3G fona = Adafruit_FONA_3G(FONA_RST);
uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout = 0);
uint8_t type;
/* ..................................................................... */


// TURBIDITY ..............................
#define turbidityPin A1 // turbidity sensor analog output to Arduino Analog Input 1
#define turbidityOffset 0
#define thermistorPin A2
/*.........................................................................*/


// pH ....................................
#define phPin A0        // pH meter Analog output to Arduino Analog Input 0
#define Offset 0.00     // deviation compensate
#define samplingInterval 20
#define printInterval 800
#define ArrayLength 40    // times of collection
int pHArray[ArrayLength]; // Store the average value of the sensor feedback
int pHArrayIndex = 0;
/* .............................................................................*/

// TEMPERATURE ..................................
int tempPin = 7;
# define tempOffset 17.0
OneWire ds(tempPin);
/*........................................................................*/

// INTERRUPTS................................
DS1337 RTC; // Create RTC object for DS1337 RTC
static uint8_t prevSecond=0; 
static DateTime interruptTime;
static uint16_t interruptInterval = 10; //Seconds. Change this to suitable value.
/* ..............................................................................  */

#define LED 8

String data = "";
float temp = -1.00; // in F
float ph = -1.00;
float turbidity = -1.00;
float thermistorTemp = -1.00;

void setup () {
     /*Initialize INT0 pin for accepting interrupts */
     PORTD |= 0x04;
     DDRD &=~ 0x04;
     Serial.begin(115200);
     RTC.begin();
     
     pinMode(4,OUTPUT);//SD Card power control pin. LOW = On, HIGH = Off
     digitalWrite(4,LOW); //Power On SD Card.

     pinMode(10, OUTPUT);
     digitalWrite(10, HIGH);
     
     fonaSerial->begin(4800);
     if (! fona.begin(*fonaSerial)) {
       Serial.println(F("Couldn't find FONA"));
       while (1);
     }
     Serial.println(F("FONA is OK"));
     Serial.print(F("Found "));
     
     Serial.print("Load SD card...");
     
     // Check if SD card can be initialized.
     if (!SD.begin(10))  //Chipselect is on pin 10
     {
        Serial.println("SD Card could not be initialized, or not found");
        return;
     }
     Serial.println("SD Card found and initialized.");
     
     
     attachInterrupt(0, INT0_ISR, LOW); //Only LOW level interrupt can wake up from PWR_DOWN
     set_sleep_mode(SLEEP_MODE_PWR_DOWN);
 
     //Enable Interrupt 
     //RTC.enableInterrupts(EveryMinute); //interrupt at  EverySecond, EveryMinute, EveryHour
     // or this
     DateTime  start = RTC.now();
     interruptTime = DateTime(start.get() + interruptInterval); //Add interruptInterval in seconds to start time
     
     // initialize digital pin 8 as an output for the LED.
     pinMode(LED, OUTPUT);
}

void loop () {
    float voltage;
    int BatteryValue;
 
    BatteryValue = analogRead(A7);
    voltage = BatteryValue * (1.1 / 1024)* (10+2)/2;  //Voltage divider
    Serial.print("Battery: ");
    Serial.println(voltage, DEC);
    
    temp = getTemp(1);
    Serial.print("Temp: ");
    Serial.println(temp, DEC);
    
    ph = getpH();
    Serial.print("pH: ");
    Serial.println(ph, DEC);
    
    turbidity = getTurbidity();
    Serial.print("turbidity: ");
    Serial.println(turbidity, DEC);
    
    thermistorTemp = getThermistorTemp();
    Serial.print("thermistorTemp: ");
    Serial.println(thermistorTemp, DEC);
    
    DateTime now = RTC.now(); //get the current date-time
    prevSecond = now.second();
    
    //|||||||||||||||||||Write to Disk|||||||||||||||||||||||||||||||||| 
    File logFile = SD.open("DATALOG.CSV", FILE_WRITE);
    Serial.println(logFile);
    if (logFile) {
      Serial.println("printing to logfile");
      // temp
      logFile.print(temp, DEC);
      logFile.print(',');
      // pH
      logFile.print(ph, DEC);
      logFile.print(',');
      // turbidity
      logFile.print(turbidity, DEC);
      logFile.print(',');
      // timestamp
      logFile.print(now.year(), DEC);
      logFile.print('/');
      logFile.print(now.month(), DEC);
      logFile.print('/');
      logFile.print(now.date(), DEC);
      logFile.print(',');
      logFile.print(now.hour(), DEC);
      logFile.print(':');
      logFile.print(now.minute(), DEC);
      logFile.print(':');
      logFile.print(now.second(), DEC);
      logFile.print(',');
      logFile.println(voltage);
      logFile.close();
    }

    //|||||||||||||||||||Write to Disk||||||||||||||||||||||||||||||||||
    
    RTC.clearINTStatus(); //This function call is a must to bring /INT pin HIGH after an interrupt.
    RTC.enableInterrupts(interruptTime.hour(),interruptTime.minute(),interruptTime.second());    // set the interrupt at (h,m,s)
    sleep_enable();      // Set sleep enable bit
    attachInterrupt(0, INT0_ISR, LOW);  //Enable INT0 interrupt (as ISR disables interrupt). This strategy is required to handle LEVEL triggered interrupt
    delay(5000);
            
    //Power Down routines
    cli(); 
    sleep_enable();
    sleep_bod_disable(); // Disable brown out detection during sleep. Saves more power
    sei();
    digitalWrite(4,HIGH); // Power Off SD Card.
    
    Serial.println("\nSleeping");
    delay(10); //This delay is required to allow print to complete
    // Shut down all peripherals like ADC before sleep. Refer Atmega328 manual
    power_all_disable(); // This shuts down ADC, TWI, SPI, Timers and USART
    sleep_cpu();         // Sleep the CPU as per the mode set earlier(power down)
    sleep_disable();     // Wakes up sleep and clears enable bit. Before this ISR would have executed
    power_all_enable();  // This enables ADC, TWI, SPI, Timers and USART
    delay(10);         // This delay is required to allow CPU to stabilize
    Serial.println("Awake from sleep");    
    digitalWrite(4,LOW); //Power On SD Card.
} 


// returns temp from one DS18B20 in C or F
float getTemp(int units) { 
  byte data[12];
  byte addr[8];

  if ( !ds.search(addr)) {// no more sensors on chain, reset search
    ds.reset_search();
    return -1000;
  }
  if ( OneWire::crc8( addr, 7) != addr[7]) {
    Serial.println("CRC is not valid!");
    return -1000;
  }
  if ( addr[0] != 0x10 && addr[0] != 0x28) {
    Serial.print("Device is not recognized");
    return -1000;
  }

  ds.reset();
  ds.select(addr);
  ds.write(0x44,1); // start conversion, with parasite power on at the end
  byte present = ds.reset();
  ds.select(addr);
  ds.write(0xBE); // Read Scratchpad
  for (int i = 0; i < 9; i++) { // we need 9 bytes
    data[i] = ds.read();
  }
  ds.reset_search();
  byte MSB = data[1];
  byte LSB = data[0];
  float tempRead = ((MSB << 8) | LSB); //using two's compliment
  float TemperatureSum = tempRead / 16;
  if (units >= 0) {
    TemperatureSum = TemperatureSum * (9/5) + 32;
  }
  
//  Serial.print("Temp: ");
//  Serial.println(TemperatureSum);
  return TemperatureSum + tempOffset;
}

// returns current pH
float getpH() {
  static unsigned long samplingTime = millis();
  static unsigned long printTime = millis();
  static float pHValue, voltage;
  if (millis() - samplingTime > samplingInterval) {
      pHArray[pHArrayIndex++] = analogRead(phPin);
      if (pHArrayIndex == ArrayLength) {
        pHArrayIndex = 0;
      }
      voltage = averageArray(pHArray, ArrayLength) * 5.0/1024;
      pHValue = 3.5 * voltage + Offset;
      samplingTime = millis();
  }
  if (millis() - printTime > printInterval) {   //Every 800 milliseconds, print a numerical, convert the state of the LED indicator
    Serial.print("Voltage:");
    Serial.println(voltage,2);
//    Serial.print("    pH value: ");
//    Serial.println(pHValue,2);
    digitalWrite(LED,digitalRead(LED)^1);
    printTime = millis();
  }
  return pHValue; 
}

// returns current turbidity/tds
float getTurbidity () {
   float sensorValue = analogRead(turbidityPin);
   Serial.println(sensorValue);
   float voltage = sensorValue * (5.0/1023.0);
   Serial.print("Turbidity voltage: ");
   Serial.println(voltage, DEC);
   float turbidity = -1.333*log(voltage) + 4.71;
   return turbidity;
}

float getThermistorTemp() {
   float sensorValue = analogRead(turbidityPin);
   float temp;
   temp = log(10000.0*((1024.0/sensorValue-1)));
   temp = 1 / (0.001129148 + (0.000234125 + (0.0000000876741 * temp * temp ))* temp);
   temp = temp - 273.15;            // Convert Kelvin to Celcius
   temp = (temp * 9.0)/ 5.0 + 32.0; // Convert Celcius to Fahrenheit
   return temp;
}


double averageArray(int* arr, int number) {
  int i;
  int max, min;
  double avg;
  long amount = 0;
  if (number <= 0) {
    Serial.println("Error: averageArray(): number can't be less than or equal to 0/n");
    return 0;
  }
  if (number < 5) {   
    for (i = 0; i < number; i++) {
      amount += arr[i];
    }
    avg = amount/number;
    return avg;
  } else {
    if (arr[0] < arr[1]) {
      min = arr[0];
      max = arr[1];
    } else {
      min = arr[1];
      max = arr[0];
    }
    for (i = 2; i < number; i++) {
      if (arr[i] < min) {
        amount += min;        //arr<min
        min = arr[i];
      } else {
        if (arr[i] > max) {
          amount += max;    //arr>max
          max = arr[i];
        } else {
          amount += arr[i]; //min<=arr<=max
        }
      } // end if
    } // end for
    avg = (double) amount / (number-2);
  } // end if
  return avg;
}

String formPOSTRequest(float pH, float temp, float tds) {
  String request = "";
  
  return request;
}




