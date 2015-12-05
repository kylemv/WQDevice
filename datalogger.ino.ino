#include <avr/sleep.h> 
#include <avr/power.h>
#include <Wire.h>
#include <OneWire.h>
#include <DS1337.h>
#include <SPI.h>
#include <SD.h>
#include <SoftwareSerial.h>
#include <String.h>

#define FONA_RX 5
#define FONA_TX 6
#define FONA_RST 7

// pH ....................................
#define phPin A0        // pH meter Analog output to Arduino Analog Input 0
#define Offset 0.00     // deviation compensate
#define samplingInterval 20
#define printInterval 800
#define ArrayLength 40    // times of collection

/* .............................................................................*/

// INTERRUPTS................................
DS1337 RTC; // Create RTC object for DS1337 RTC
static uint8_t prevSecond = 0; 
static DateTime interruptTime;
static uint16_t interruptInterval = 5; //Seconds. Change this to suitable value.
/* ..............................................................................  */

#define LED 9

#include <SoftwareSerial.h>
SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
SoftwareSerial *fonaSerial = &fonaSS;

void setup() {
     /*Initialize INT0 pin for accepting interrupts */
     PORTD |= 0x04;
     DDRD &=~ 0x04;
     
     Serial.begin(115200);
     RTC.begin();
     
     pinMode(4,OUTPUT);//SD Card power control pin. LOW = On, HIGH = Off
     digitalWrite(4,LOW); //Power On SD Card. 
     
     pinMode(9, OUTPUT);
     digitalWrite(LED, HIGH);
     delay(100);
     digitalWrite(LED, LOW);
     delay(100);
     digitalWrite(LED, HIGH);
     delay(100);
     digitalWrite(LED, LOW);
     delay(100);
     digitalWrite(LED, HIGH);
     
     pinMode(10, OUTPUT);
     digitalWrite(10, HIGH);
     
     Serial.println(F("Starting up fona"));
     fonaSerial->begin(4800);
     pinMode(FONA_RST, OUTPUT);
     digitalWrite(FONA_RST, HIGH);
     delay(10);
     digitalWrite(FONA_RST, LOW);
     delay(100);
     digitalWrite(FONA_RST, HIGH);

     // give 7 seconds to reboot
     delay(7000);
     int answer = sendATcommand("AT", "OK", 2000);
     if (answer != 1) {
       Serial.println(F("Couldn't find FONA"));
       return;
     } 
     Serial.println("FONA is OK");
     
     delay(100);

     answer = sendATcommand("AT+CGSOCKCONT=1,\"IP\",\"wholesale\",\"0.0.0.0\",0,0", "OK", 2000);
     if (answer != 1) {
       Serial.println(F("CGSOCKCONT failed"));
     }

     delay(500);
     answer = sendATcommand("AT+CSOCKAUTH=1,2,\"SIMCOM\",\"123\"", "OK", 2000);
     if (answer != 1) {
       Serial.println(F("CGSOCKAUTH failed"));
     }
     delay(1500);
     
     attachInterrupt(0, INT0_ISR, LOW); //Only LOW level interrupt can wake up from PWR_DOWN
     set_sleep_mode(SLEEP_MODE_PWR_DOWN);
 
     // Enable Interrupt 
     DateTime  start = RTC.now();
     interruptTime = DateTime(start.get() + interruptInterval); //Add interruptInterval in seconds to start time
     
           // Check if SD card can be initialized.
     if (!SD.begin(10))  //Chipselect is on pin 10
     {
        Serial.println(F("SD Card could not be initialized, or not found"));
        delay(100);
        Serial.println(F("Trying again"));
        if (!SD.begin(10))  //Chipselect is on pin 10
        {
          Serial.println(F("SD Card could not be initialized, or not found"));
          delay(100);
          Serial.println(F("Trying again"));
          if (!SD.begin(10))  //Chipselect is on pin 10
          {
            Serial.println(F("SD Card could not be initialized, or not found"));
            delay(100);
            return;   
          }
        }
     }
     digitalWrite(LED, HIGH);
     delay(100);
     digitalWrite(LED, LOW);
     delay(100);
     digitalWrite(LED, HIGH);
     delay(100);
     digitalWrite(LED, LOW);
     delay(100);
     
     Serial.println(F("SD Card found and initialized."));
}

void loop() {
//    float voltage = analogRead(A7) * (1.1 / 1024)* (10+2)/2;  // Voltage divider
    
    float temp = getTemp();
//    Serial.print("Temp: ");
//    Serial.println(temp, DEC);
    
    float ph = getpH();
//    Serial.print("pH: ");
//    Serial.println(ph, DEC);
    
    sendPOSTRequest(ph, temp);
    writeToSDCard(ph, temp);
    goToSleep();
    
}

void goToSleep() {
    RTC.clearINTStatus(); //This function call is a must to bring /INT pin HIGH after an interrupt.
    RTC.enableInterrupts(interruptTime.hour(),interruptTime.minute(),interruptTime.second());    // set the interrupt at (h,m,s)
    sleep_enable();      // Set sleep enable bit
    attachInterrupt(0, INT0_ISR, LOW);  //Enable INT0 interrupt (as ISR disables interrupt). This strategy is required to handle LEVEL triggered interrupt
                  
    //Power Down routines
    cli(); 
    sleep_enable();
    sleep_bod_disable(); // Disable brown out detection during sleep. Saves more power
    sei();
    digitalWrite(4,HIGH); // Power Off SD Card.
    blinkLED(1000);
    Serial.println(F("\nSleeping"));
    delay(10); //This delay is required to allow print to complete
    // Shut down all peripherals like ADC before sleep. Refer Atmega328 manual
    power_all_disable(); // This shuts down ADC, TWI, SPI, Timers and USART
    sleep_cpu();         // Sleep the CPU as per the mode set earlier(power down)
    sleep_disable();     // Wakes up sleep and clears enable bit. Before this ISR would have executed
    power_all_enable();  // This enables ADC, TWI, SPI, Timers and USART
    delay(10);         // This delay is required to allow CPU to stabilize
    Serial.println(F("Awake from sleep"));    
    digitalWrite(4,LOW); //Power On SD Card.
    delay(1500);
    blinkLED(1000);
}

void writeToSDCard(float phVal, float tempVal) {
    DateTime now = RTC.now(); //get the current date-time
    File logFile = SD.open("DATALOG.CSV", FILE_WRITE);
    Serial.println(logFile);

    if (logFile) {
      Serial.println(F("printing to logfile"));
      // temp
      logFile.print(tempVal, DEC);
      logFile.print(',');
      // pH
      logFile.print(phVal, DEC);
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
      logFile.close();
      blinkLED(100);
    }
}

void sendPOSTRequest(float phVal, float tempVal) {
  char request[160] = "POST /data/newData HTTP/1.1\r\nContent-Length: 52\r\nHost: vaccine.cs.umd.edu\r\nAccept: text/html\r\n\r\ndevice_id=1&data_type=water&ph=";
  char phArr[5];
  char tempArr[5];
  dtostrf(phVal,4,1,phArr);
  dtostrf(tempVal,4,1,tempArr);
  strcat(request, phArr);
  strcat(request, "&temperature=");
  strcat(request, tempArr);
//  Serial.print(F("REQUEST IS: "));
//  Serial.println(request);
  // gets the SSL stack
  int answer = sendATcommand("AT+CHTTPSSTART", "OK", 10000);
  if (answer == 1) { 
    // Opens the HTTP session
    answer = sendATcommand("AT+CHTTPSOPSE=\"vaccine.cs.umd.edu\",80,1", "OK", 30000);
    if (answer == 1) {
      // Stores and sends the request
      answer = sendATcommand("AT+CHTTPSSEND=155", ">", 5000);
      if (answer == 1) {
        answer = sendATcommand(request, "OK", 5000); 
        if (answer == 1) { 
          // request the url
          answer = sendATcommand("AT+CHTTPSSEND", "OK", 5000);
          blinkLED(10);
        } else {
          Serial.println(F("Error sending request to FONA"));
        }
      } else {
        Serial.println(F("Error in sending aux_str to FONA"));
      }
    } else {
       Serial.println(F("Error in opening HTTP session"));   
    }
  } else {
    Serial.println(F("Error in opening SSL stack"));
  }
  sendATcommand("AT+CHTTPSCLSE", "OK", 5000);  // close http session
  sendATcommand("AT+CHTTPSSTOP", "OK", 5000);  // stop http stack
  delay(5000);
  digitalWrite(FONA_RST, HIGH);
  delay(10);
  digitalWrite(FONA_RST, LOW);
  delay(100);
  digitalWrite(FONA_RST, HIGH);
  delay(7000);
  return;
}

int8_t sendATcommand(char* ATcommand, char* expected_answer, unsigned int timeout){
    unsigned long previous;
    
    delay(300); // Delay to be sure no passed commands interfere
    
    while( fonaSS.available() > 0) fonaSS.read();    // Wait for clean input buffer
    
    fonaSS.println(ATcommand); // Send the AT command 
 
    previous = millis();
    
    // this loop waits for the answer
    do {
        // if there are data in the UART input buffer, reads it and checks for the answer
        if(fonaSS.available() != 0) {
          
              char a = fonaSS.read();
              char b = fonaSS.read();
              Serial.print(a);
              Serial.print(b);
              if ((a == 'K' || b == 'K') || (a == '>' || b == '>')) {
                return 1;
              }
        }
    // Waits for the answer with time out
    } while((millis() - previous) < timeout);
 
    return 0;
}

// returns temp from one DS18B20 in C or F
float getTemp() { 
  OneWire ds(8);
  byte data[12];
  byte addr[8];


  if ( !ds.search(addr)) {// no more sensors on chain, reset search
    ds.reset_search();
    return -1000;
  }
  
  ds.reset();
//  ds.select((uint8_t*) "0x28, 0x9D, 0xF2, 0x97, 0x05, 0x00, 0x00, 0x9");
  ds.select(addr);
  ds.write(0x44,1); // start conversion, with parasite power on at the end
  byte present = ds.reset();
  ds.select(addr);
//  ds.select((uint8_t*) "0x28, 0x9D, 0xF2, 0x97, 0x05, 0x00, 0x00, 0x9");
  ds.write(0xBE); // Read Scratchpad
  
  data[0] = ds.read();
  data[1] = ds.read();
  data[2] = ds.read();
  data[3] = ds.read();
  data[4] = ds.read();
  data[5] = ds.read();
  data[6] = ds.read();
  data[7] = ds.read();
  data[8] = ds.read();
  ds.reset_search();
  float TemperatureSum = ((data[1] << 8) | data[0]) / 16.0;  //using two's compliment
  TemperatureSum = TemperatureSum * (9/5) + 32; // convert to F
  
  return TemperatureSum + 17.0;
}

//// returns current pH
float getpH() {
  return (float) 3.5 * (analogRead(phPin) * 5.0/1024) + Offset;
}

//Interrupt service routine for external interrupt on INT0 pin conntected to DS1337 /INT
void INT0_ISR()
{
    // Keep this as short as possible. Possibly avoid using function calls
    detachInterrupt(0); 
    interruptTime = DateTime(interruptTime.get() + interruptInterval);  //decide the time for next interrupt, configure next interrupt  
}

void blinkLED(int seconds) {
  digitalWrite(LED, HIGH);
  delay(seconds);
  digitalWrite(LED, LOW);
  delay(seconds);
  digitalWrite(LED, HIGH);
  delay(seconds);
  digitalWrite(LED, LOW);
}
