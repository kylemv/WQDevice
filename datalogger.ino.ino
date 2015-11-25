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

#define turbidityPin A1 // turbidity sensor analog output to Arduino Analog Input 1

// pH .....................................
#define phPin A0        // pH meter Analog output to Arduino Analog Input 0
#define Offset 0.00     // deviation compensate
#define samplingInterval 20
#define printInterval 800
#define ArrayLength 40    // times of collection
int pHArray[ArrayLength]; // Store the average value of the sensor feedback
int pHArrayIndex = 0;    

#define FONA_RX 2
#define FONA_TX 3
#define FONA_RST 4

#define LED 8

int tempPin = 7;

DS1337 RTC; //Create RTC object for DS1337 RTC
static uint8_t prevSecond=0; 
static DateTime interruptTime;

static uint16_t interruptInterval = 10; //Seconds. Change this to suitable value.
OneWire ds(tempPin);


String data = "";
float temp = -1.00; // in F
float ph = -1.00;
float tds = -1.00;

void setup () {
     /*Initialize INT0 pin for accepting interrupts */
     PORTD |= 0x04;
     DDRD &=~ 0x04;

//     Wire.begin();
     Serial.begin(115200);
     RTC.begin();
     
     pinMode(4,OUTPUT);//SD Card power control pin. LOW = On, HIGH = Off
     digitalWrite(4,LOW); //Power On SD Card.

     pinMode(10, OUTPUT);
     digitalWrite(10, HIGH);
     
     // initialize digital pin 8 as an output for the LED.
     pinMode(LED, OUTPUT);
     
     Serial.print("Load SD card...");
     
     // Check if SD card can be intialized.
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
}

void loop () {
    
    float voltage;
    int BatteryValue;
 
    BatteryValue = analogRead(A7);
    voltage = BatteryValue * (1.1 / 1024)* (10+2)/2;  //Voltage divider
    temp = getTemp(1);
    Serial.println("Temp: ");
    Serial.print(temp, DEC);
    ph = getpH();
    Serial.println("pH: ");
    Serial.print(ph, DEC);
    tds = getTurbidity();
    data = formPOSTRequest(ph, temp, tds);
 
    DateTime now = RTC.now(); //get the current date-time  
    now.m = now.month() + 8;
    now.yOff = now.year() - 2000 + 14;
    now.d = now.date() + 16;
    if((now.second()) !=  prevSecond )
    {
    //print only when there is a change
    Serial.print(now.year(), DEC);
    Serial.print('/');
    Serial.print(now.month(), DEC);
    Serial.print('/');
    Serial.print(now.date(), DEC);
    Serial.print(' ');
    Serial.print(now.hour(), DEC);
    Serial.print(':');
    Serial.print(now.minute(), DEC);
    Serial.print(':');
    Serial.print(now.second(), DEC);
    Serial.print("   ");
    Serial.print(voltage);
    Serial.print(" V");
    Serial.println();

    }
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
    digitalWrite(LED, HIGH);
    delay(100);
    digitalWrite(LED, LOW);
    delay(100);
    digitalWrite(LED, HIGH);
    delay(100);
    digitalWrite(LED, LOW);
    delay(100);
    }

    //|||||||||||||||||||Write to Disk||||||||||||||||||||||||||||||||||
    
    RTC.clearINTStatus(); //This function call is a must to bring /INT pin HIGH after an interrupt.
    RTC.enableInterrupts(interruptTime.hour(),interruptTime.minute(),interruptTime.second());    // set the interrupt at (h,m,s)
    sleep_enable();      // Set sleep enable bit
    attachInterrupt(0, INT0_ISR, LOW);  //Enable INT0 interrupt (as ISR disables interrupt). This strategy is required to handle LEVEL triggered interrupt
    
    
    ////////////////////////END : Application code //////////////////////////////// 
   
    
    //\/\/\/\/\/\/\/\/\/\/\/\/Sleep Mode and Power Down routines\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\
            
    //Power Down routines
    cli(); 
    sleep_enable();
    sleep_bod_disable(); // Disable brown out detection during sleep. Saves more power
    sei();
//       
    digitalWrite(4,HIGH); //Power Off SD Card.
    
    Serial.println("\nSleeping");
    delay(10); //This delay is required to allow print to complete
    // Shut down all peripherals like ADC before sleep. Refer Atmega328 manual
    power_all_disable(); // This shuts down ADC, TWI, SPI, Timers and USART
//    power_adc_disable();
//    power_spi_disable();
//    power_timer0_disable();
//    power_timer1_disable();
//    power_timer2_disable();
//    power_twi_disable();
//    power_usart0_disable();
    sleep_cpu();         // Sleep the CPU as per the mode set earlier(power down)
    sleep_disable();     // Wakes up sleep and clears enable bit. Before this ISR would have executed
    power_all_enable();  // This enables ADC, TWI, SPI, Timers and USART
    delay(10);         // This delay is required to allow CPU to stabilize
    Serial.println("Awake from sleep");    
    digitalWrite(4,LOW); //Power On SD Card.

    //\/\/\/\/\/\/\/\/\/\/\/\/Sleep Mode and Power Saver routines\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\

} 

  
//Interrupt service routine for external interrupt on INT0 pin conntected to DS1337 /INT
void INT0_ISR()
{
    // Keep this as short as possible. Possibly avoid using function calls
    detachInterrupt(0); 
    interruptTime = DateTime(interruptTime.get() + interruptInterval);  //decide the time for next interrupt, configure next interrupt  
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
  
  Serial.print("Temp: ");
  Serial.println(TemperatureSum);
  return TemperatureSum;
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
    Serial.print(voltage,2);
    Serial.print("    pH value: ");
    Serial.println(pHValue,2);
    digitalWrite(LED,digitalRead(LED)^1);
    printTime = millis();
  }
  return pHValue; 
}

// returns current turbidity/tds
float getTurbidity () {
   int num = analogRead(turbidityPin);
   float voltage = num/1023;
   float turbidity = -1.333*log(voltage) + 4.71;
   return turbidity;
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




