///////////////////////////////////////////////////////////////////////////////
#define RF69_COMPAT      0   // define this to use the RF69 driver i.s.o. RF12 
//                           // The above flag must be set similarly in RF12.cpp
//                           // and RF69_avr.h
///////////////////////////////////////////////////////////////////////////////
#include <JeeLib.h>
#include <avr/sleep.h>
#include <OneWire.h>
#define AIO2 9 // d9
#define SALUSID 16
#define SALUSFREQUENCY 1669 // JeeLink 69c (Red)
#define ON  1
#define OFF 2
#define TEMPCHECK 60        // Check the temperatures each minute

#define REG_BITRATEMSB 0x03 // RFM69 only, 0x02, // BitRateMsb, data rate = 49,261 khz
#define REG_BITRATELSB 0x04 // RFM69 only, 0x8A, // BitRateLsb divider = 32 MHz / 650 == 49,230 khz
#define REG_BITFDEVMSB 0x05 // RFM69 only, 0x02, // FdevMsb = 45 KHz
#define REG_BITFDEVLSB 0x06 // RFM69 only, 0xE1, // FdevLsb = 45 KHz

OneWire  ds(PD7); // DIO4
/*
 * This code monitors dimensions of a home heating system and exercises
 * some limited control. A design imperative is for any Jee equipment to
 * fail soft with the more typical central heating controls continuing to  
 * manage the heating system. During such failure some features may not
 * be operational. We assume that a Salus thermostat is controlling the
 * heating demand. When doing nothing else this Jeenode will monitor for
 * RF Salus thermostat commands and retransmit them in RF12 format to the
 * data collection server. The Salus equipment has direct control of the
 * heating demand but this Jeenode is able to send Salus commands,
 * typically OFF commands to optimise heating performance and gas consumption.
 * 
 * In addition to the above various temperatures around the heating system 
 * will be collected as below. Where the central heating return approaches 
 * the boiler feed temperature a Salus command will be issued to cancel
 * the central heating demand. This temperature difference is where the tuning
 * of gas consumption may be acheived. 
 * It should be noted that the Salus thermostat will repeat its instructions 
 * to the boiler based on temperature and every 10 minutes. If this sketch
 * cancels the heating demand the demand will be reasserted by the thermostat if 
 * conditions dictate.
 * 
 * If the Salus thermostat fails to deliver its demand status on its regular
 * schedule then this sketch will cancel the central heating demand.
 */

unsigned long salusMillis;
static unsigned long salusTimeout = 21 * 60 * 1000ul;
unsigned int loopCount;
byte ColdFeed[8] = {0x28,0x9E,0x77,0x37,0x03,0x00,0x00,0xAA};
byte BoilerFeed[8] = {0x28,0x86,0x39,0x4E,0x04,0x00,0x00,0x5A};
byte CentralHeatingReturn[8] = {0x28,0x7F,0xCA,0x4D,0x4,0x0,0x0,0xDE};
byte TankCoilReturn[8] = {0x28,0x7F,0xC6,0x4D,0x04,0x00,0x00,0xFF};
byte HotFeed[8] = {0x28,0x53,0x4F,0x4E,0x04,0x00,0x00,0x84};

byte addr[8];
byte needOff = false;
byte salusOff[] = {SALUSID, OFF, SALUSID | OFF, 90};
byte elapsed = 0;

ISR(WDT_vect) { Sleepy::watchdogEvent(); }
//
/////////////////////////////////////////////////////////////////////////////////////
struct payload{                                                                    //
byte attempts: 4;     // transmission attempts
byte count: 4;        // packet count
byte salusAddress;
byte salusCommand;
unsigned int salusNoise;
unsigned int ColdFeed;
unsigned int BoilerFeed;
unsigned int CentralHeatingReturn;
unsigned int TankCoilReturn;
unsigned int HotFeed;
} payload;                                                                         //
/////////////////////////////////////////////////////////////////////////////////////

unsigned int getTemp(void)
{
  byte i;
  byte present = 0;
  byte data[12]; 
  
//  Serial.print("  Data = ");
//  Serial.print(present,HEX);
//  Serial.print(" ");

  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
//    Serial.print(data[i], HEX);
//    Serial.print(" ");
  }
//  Serial.print(" CRC=");
//  Serial.print(OneWire::crc8(data, 8), HEX);
 // Serial.println();

  // convert the data to actual temperature

  int raw = (data[1] << 8) | data[0];
//  raw = raw << 3; // 9 bit resolution default
  raw = raw * 10;
  raw = raw >> 4;
  return (raw); // return t*10
//  return ((data[1] << 8) + data[0]); // return t*16
}
// Ten off wired DS18B20 from Alibaba
// 28 7F C6 4D 4 00 00 FF Tank Coil
// 28 7F CA 4D 4 00 00 DE Central Heating Return
// 28 53 4F 4E 4 00 00 84 Tank Stat
// 28 E8 AC 4E 4 00 00 EF
// 28 26 E6 4D 4 00 00 BF
// 28 86 39 4E 4 00 00 5A Boiler Feed
// 28 A0 68 4E 4 00 00 06
// 28 4E D3 4D 4 00 00 8B
// 28 F3 D1 4D 4 00 00 45
// 28 C9 C5 4D 4 00 00 04
//////////////////////////

#define ACK_TIME        20  // number of milliseconds - 1 to wait for an ack
#define RETRY_LIMIT      7
byte NodeID = 31;

static byte sendACK() {
  for (byte t = 1; t <= RETRY_LIMIT; t++) {  
      rf12_sleep(RF12_WAKEUP);
//      while (!rf12_canSend())
      delay(t * t);                   // Increasing the gap between retransmissions
      payload.attempts = t;
      rf12_sendStart(RF12_HDR_ACK, &payload, sizeof payload);
      byte acked = waitForAck(t * t); // Wait for increasingly longer time for the ACK
      rf12_sleep(RF12_SLEEP);
        if (acked) {
          return t;
        } else {
            Serial.print(".");
            Serial.flush();
        }
     }
     return 0; 
  }
static byte waitForAck(byte t) {
    MilliTimer ackTimer;
    while (!ackTimer.poll(ACK_TIME + t)) {
        if (rf12_recvDone() && rf12_crc == 0 &&
                // see http://talk.jeelabs.net/topic/811#post-4712
                rf12_hdr == (RF12_HDR_DST | RF12_HDR_CTL | NodeID))
            return 1;
        set_sleep_mode(SLEEP_MODE_IDLE);   // Wait a while for the reply?
        sleep_mode();
    }
    return 0;
}

static void printOneChar (char c) {
    Serial.print(c);
}

static void showNibble (byte nibble) {
    char c = '0' + (nibble & 0x0F);
    if (c > '9')
        c += 7;
    Serial.print(c);
}

static void showByte (byte value) {
//    if (config.output & 0x1) {
        showNibble(value >> 4);
        showNibble(value);
//    } else
//        Serial.print((word) value, DEC);
}
static void showWord (unsigned int value) {
//    if (config.output & 0x1) {
        showByte (value >> 8);
        showByte (value);
//    } else
//        Serial.print((word) value);    
}


void setup () {
  delay(100);          // Delay on startup to avoid ISP/RFM12B interference.
  Serial.begin(57600);
#if RF69_COMPAT
  Serial.print("RFM69x ");
#else
  Serial.print("RFM12x ");
#endif
  Serial.print(SALUSFREQUENCY);  
  Serial.print(" Heating monitor:");
  rf12_configDump();
  Serial.flush();
  payload.BoilerFeed = ~0;
  payload.salusAddress = ~0;          // Until we know better
  payload.salusCommand = ON;          // ditto
  salusMillis = millis() + salusTimeout;
  
/*
  pinMode(17, OUTPUT);      // Set the pin, AIO4 - Power the DS18B20's
  digitalWrite(17, HIGH);   // Power up the DS18B20's
  
/// Configure the DS20B18 ///
  ds.reset();               // Set for 9 bit measurements //
  ds.skip();                // Next command to all devices
  ds.write(0x4E);           // Write to Scratch Pad
  ds.write(0x7F);           // Set T(h)
  ds.write(0x80);           // Set T(l)
  ds.write(0x1F);           // Set Config
/// Copy config to on-chip EEProm
  ds.reset();               // Set for 9 bit measurements //
  ds.skip();                // Next command to all devices
  ds.write(0x48);           // Set Config
  delay(10);                // Wait for copy to complete
*/
  }
void loop () {
/*
 * Setup to receive Salus transmissions
 */
    rf12_initialize (SALUSID, RF12_868MHZ, 212, SALUSFREQUENCY);  // 868.360khz
    rf12_sleep(RF12_SLEEP);                                       // Sleep while we tweak things
/*    
    for (byte i = 0; i < 8; i++) {                                // Paint some buffer
        rf12_buf[i] = 0xFF;
    }
*/    
#if RF69_COMPAT
    RF69::control(REG_BITRATEMSB | 0x80, 0x34);                   // 2.4kbps
    RF69::control(REG_BITRATELSB | 0x80, 0x15);
    RF69::control(REG_BITFDEVMSB | 0x80, 0x04);                   // 75kHz freq shift
    RF69::control(REG_BITFDEVLSB | 0x80, 0xCE);
    rf12_sleep(RF12_WAKEUP);                                      // All set, wake up radio
#else
    rf12_control(0xC040);                                         // set low-battery level to 2.2V
    rf12_control(RF12_DATA_RATE_2);                               // 0xC691 app 2.4kbps
    rf12_control(0x9830);                                         // 75khz freq shift
    rf12_sleep(RF12_WAKEUP);                                      // Wake up radio
#endif
    rf12_recvDone();                                              // Enter receive mode
    
    elapsed = elapsed + (TEMPCHECK - (Sleepy::idleSomeTime(TEMPCHECK)));  // Check temperatures every minute

#if !RF69_COMPAT
    delay(10);                                                            // Wait for the data to be available
    if (rf12_buf[4] == 90) {
#else
    if (rf12_recvDone()) {
#endif
        if ((rf12_buf[0] == 212 && ((rf12_buf[1] | rf12_buf[2]) == rf12_buf[3]) 
          && rf12_buf[4] == 90/* && rf12_buf[1] == SALUSID*/)) {          // Capture all Salus packets
            rf12_buf[4] = ~90;
            salusMillis = millis() + salusTimeout;
            rf12_sleep(RF12_SLEEP);
            Serial.print("Salus: ");
            Serial.print(rf12_buf[1]);
            Serial.print(",");
            Serial.println(rf12_buf[2]);
            Serial.flush();
            elapsed = ~0;                                             // Trigger a transmit
            payload.salusAddress = rf12_buf[1];   
            payload.salusCommand = rf12_buf[2];   
            Sleepy::loseSomeTime(50 * 29);  // Salus sends 30 packets spaced at 50ms so wait for the redundant packets to pass us by
        } else {
            payload.salusNoise++;
            for (byte i = 0; i < 8; i++) {;
                showByte(rf12_buf[i]);
                printOneChar(' ');
            }
            Serial.println();
        }
    }

/* DEBUG
            for (byte i = 0; i < 8; i++) {;
                showByte(rf12_buf[i]);
                printOneChar(' ');
            }
            Serial.println();
*/
    if (((millis() > salusMillis) || needOff)) {                  // Is a Salus Off required?
        needOff = false;
        salusMillis = millis() + salusTimeout;
        Serial.println("Sending Salus off");
        rf12_sleep(RF12_WAKEUP);
//        while (!rf12_canSend())
        rf12_skip_hdr();                                          // Omit Jeelib header 2 bytes on transmission
        rf12_sendStart(0, &salusOff, 4);                          // Transmit the Salus off command
        rf12_sendWait(1);                                         // Wait for transmission complete
        rf12_sleep(RF12_SLEEP);                                   // Sleep the radio
        payload.salusAddress = 0;                                 // Note the prolonged loss of contact
        payload.salusCommand = OFF | 0x80;                        // Update the Jee world status
    }                                                             // 0x80 indicates Jeenode commanded off
    if (elapsed >= TEMPCHECK) {
        /*
        digitalWrite(17, HIGH);                                       // Power up the DS18B20's
        ds.reset();
        ds.skip();                                                    // Next command to all devices
        ds.write(0x44);                                               // Start all temperature conversions.
        Sleepy::loseSomeTime(100);                                    // Wait for the data to be available

        ds.reset();
        ds.select(ColdFeed);    
        ds.write(0xBE);                                               // Read Scratchpad
        payload.ColdFeed = getTemp();
        Serial.println(payload.ColdFeed);

        ds.reset();
        ds.select(BoilerFeed);    
        ds.write(0xBE);                                               // Read Scratchpad
        payload.BoilerFeed = getTemp();
        Serial.println(payload.BoilerFeed);

        ds.reset();
        ds.select(CentralHeatingReturn);    
        ds.write(0xBE);                                               // Read Scratchpad
        payload.CentralHeatingReturn = getTemp();
        Serial.println(payload.CentralHeatingReturn);

        ds.reset();
        ds.select(TankCoilReturn);    
        ds.write(0xBE);                                               // Read Scratchpad
        payload.TankCoilReturn = getTemp();
        Serial.println(payload.TankCoilReturn);

        ds.reset();
        ds.select(HotFeed);    
        ds.write(0xBE);                                               // Read Scratchpad
        payload.HotFeed = getTemp();

        digitalWrite(17, LOW);                                        // Power down the DS18B20's    
    
        Serial.println(payload.HotFeed);
        Serial.println();
        Serial.flush();
        */
        if ((payload.TankCoilReturn + 2) > payload.BoilerFeed) needOff = true;
    
        payload.count++;
        if (NodeID = rf12_configSilent()) {
            Serial.print("Node ");
            Serial.print(NodeID);
            Serial.print(" sending to JeeNet #");
            Serial.print(payload.count);
            Serial.print(" ");
            Serial.flush();
            
            payload.attempts = sendACK();
            
            if(payload.attempts) { 
                Serial.print(payload.attempts);
                Serial.println(" attempts");
            } else {
                Serial.println(" aborted");
            }
            Serial.flush();             
        } else {
              while( true ){
                rf12_sleep(RF12_SLEEP);
                Serial.println("RF12 eeprom not valid, run RF12Demo");
                Serial.flush();
                Sleepy::idleSomeTime(60);
              }  
        }
        elapsed = 0; 
    }
    Serial.print("Looping ");
    Serial.println(++loopCount);
    Serial.flush();
} // Loop

