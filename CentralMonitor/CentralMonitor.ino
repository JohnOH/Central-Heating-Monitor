///////////////////////////////////////////////////////////////////////////////
#define RF69_COMPAT      1   // define this to use the RF69 driver i.s.o. RF12 
//                           // The above flag must be set similarly in RF12.cpp
//                           // and RF69_avr.h
///////////////////////////////////////////////////////////////////////////////
#include <JeeLib.h>
#include <avr/sleep.h>
#include <OneWire.h>
#define AIO2 9 // d9
#define SALUSID 16
#define ON  1
#define OFF 2
#define REG_BITRATEMSB 0x03  // RFM69 only, 0x02, // BitRateMsb, data rate = 49,261 khz
#define REG_BITRATELSB 0x04  // RFM69 only, 0x8A, // BitRateLsb divider = 32 MHz / 650 == 49,230 khz
#define REG_BITFDEVMSB 0x05  // RFM69 only, 0x02, // FdevMsb = 45 KHz
#define REG_BITFDEVLSB 0x06  // RFM69 only, 0xE1, // FdevLsb = 45 KHz


unsigned long salusMillis;
unsigned long salusTimeout = 21 * 60 * 1000ul;
unsigned int loopCount;
byte ColdFeed[8] = {0x28,0x9E,0x77,0x37,0x03,0x00,0x00,0xAA};
byte BoilerFeed[8] = {0x28,0x86,0x39,0x4E,0x04,0x00,0x00,0x5A};
byte CentralHeatingReturn[8] = {0x28,0x7F,0xCA,0x4D,0x4,0x0,0x0,0xDE};
byte TankCoilReturn[8] = {0x28,0x7F,0xC6,0x4D,0x04,0x00,0x00,0xFF};
byte HotFeed[8] = {0x28,0x53,0x4F,0x4E,0x04,0x00,0x00,0x84};

byte addr[8];
byte needOff  = false;
byte salusOff[] = {SALUSID, OFF, SALUSID | OFF, 90};
byte elapsed = 0;

ISR(WDT_vect) { Sleepy::watchdogEvent(); }
//
/////////////////////////////////////////////////////////////////////////////////////
struct payload{                                                                    //
byte count: 4;        // packet count
byte retries: 4;      // transmission attempts
byte salusCommand;
unsigned ColdFeed;
unsigned int BoilerFeed;
unsigned int CentralHeatingReturn;
unsigned int TankCoilReturn;
unsigned int HotFeed;
} payload;                                                                         //
/////////////////////////////////////////////////////////////////////////////////////
// wait a few milliseconds for proper ACK to me, return true if indeed received
#define ACK_TIME        20  // number of milliseconds to wait for an ack
#define RETRY_LIMIT      7

void setup () {
  delay(100);          // Delay on startup to avoid ISP/RFM12B interference.
  Serial.begin(57600);
  Serial.print("Heating monitor:");
  rf12_configDump();
  Serial.flush();
  payload.BoilerFeed = ~0;
  payload.salusCommand = ON;
  salusMillis = millis() + salusTimeout;
  
  }
void loop () {
/*
 * Setup to receive Salus transmissions
 */
    rf12_initialize (SALUSID, RF12_868MHZ, 212, 1652);            // 868.260khz
//    rf12_sleep(RF12_SLEEP);                                       // Sleep while we tweak things
#if RF69_COMPAT
    RF69::control(REG_BITRATEMSB | 0x80, 0x34);                   // 2.4kbps
    RF69::control(REG_BITRATELSB | 0x80, 0x15);
    RF69::control(REG_BITFDEVMSB | 0x80, 0x04);                   // 75kHz freq shift
    RF69::control(REG_BITFDEVLSB | 0x80, 0xCE);
#else
    rf12_control(0xC040);                                         // set low-battery level to 2.2V
    rf12_control(RF12_DATA_RATE_2);                               // 2.4kbps
    rf12_control(0x9840);                                         // 75khz freq shift
#endif
    rf12_recvDone();                                              // Enable receiver
//    rf12_sleep(RF12_SLEEP);                                       // Sleep while we tweak things
    
//    delay(2500);
    Serial.println("Starting");
    Serial.flush();
/*
    delay(10);
        Serial.println(RF69::interruptCount);
    delay(100);

*/
//    rf12_sleep(RF12_WAKEUP);                                      // All set, wake up radio    

 //   delay(10);
 //       Serial.println((RF69::control(0x28, 0)), HEX); // Prints out Register value
 //       Serial.println(RF69::interruptCount);
 //   delay(100);

    // The idleSomeTime code disables millis() to quiet down interrupts
    // Interrupts are still enabled and a packet arrival will wake us up.
    elapsed = elapsed + (60 - (Sleepy::idleSomeTime(60)));        // Check temperatures every minute

    if (rf12_recvDone()) {
        Serial.println(RF69::interruptCount);
        Serial.flush();
        if ((rf12_buf[0] == 212 && ((rf12_buf[1] | rf12_buf[2]) == rf12_buf[3]) // Split into two if's no joy
          && rf12_buf[4] == 90 && rf12_buf[1] == SALUSID)) {
            // It is a packet from our Salus thermostat!
            salusMillis = millis() + salusTimeout;
//            rf12_sleep(RF12_SLEEP);
            Serial.print(elapsed);
            Serial.print(" Salus: ");
            Serial.print(rf12_buf[1]);
            Serial.print(",");
            Serial.println(rf12_buf[2], HEX);
            Serial.flush();
            elapsed = ~0;                                             // Trigger a transmit
            payload.salusCommand = rf12_buf[2]; 
            Sleepy::loseSomeTime(50 * 29);  // Salus sends 30 packets 50ms apart, wait for the redundant packets to pass us by.
          }  
    }
    Serial.print("Loop ");
    Serial.println(++loopCount);
    Serial.flush();
} // Loop

