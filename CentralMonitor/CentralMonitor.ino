///////////////////////////////////////////////////////////////////////////////
#define RF69_COMPAT      0   // define this to use the RF69 driver i.s.o. RF12 
//                           // The above flag must be set similarly in RF12.cpp
//                           // and RF69_avr.h
///////////////////////////////////////////////////////////////////////////////
#include <JeeLib.h>
#include <util/crc16.h>
#include <avr/sleep.h>
#include <OneWire.h>
#define AIO2 9 // d9
#define SALUSID 16
#define SALUSFREQUENCY 1660
#define ON  1
#define OFF 2
#define TEMPCHECK 60        // Check the temperatures every 60 seconds

#define REG_BITRATEMSB 0x03 // RFM69 only, 0x02, // BitRateMsb, data rate = 49,261 khz
#define REG_BITRATELSB 0x04 // RFM69 only, 0x8A, // BitRateLsb divider = 32 MHz / 650 == 49,230 khz
#define REG_BITFDEVMSB 0x05
#define REG_BITFDEVLSB 0x06

#if DEBUG
#define D(x) x
#else
#define D(x)
#endif

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
byte ColdFeed[8] = {0x28,0x53,0x4F,0x4E,0x04,0x00,0x00,0x84};
byte BoilerFeed[8] = {0x28,0x86,0x39,0x4E,0x04,0x00,0x00,0x5A};
byte CentralHeatingReturn[8] = {0x28,0x7F,0xCA,0x4D,0x4,0x0,0x0,0xDE};
byte TankCoilReturn[8] = {0x28,0x7F,0xC6,0x4D,0x04,0x00,0x00,0xFF};

byte addr[8];
byte needOff = false;
byte salusOff[] = {SALUSID, OFF, SALUSID | OFF, 90};
byte WatchSALUS = false;
unsigned int elapsed = 0;
unsigned int maxCH = 6000;
unsigned int maxBoiler = 6500;

ISR(WDT_vect) { Sleepy::watchdogEvent(); }
//
/////////////////////////////////////////////////////////////////////////////////////
struct payload{                                                                    //
byte command;         // Last command received in ACK
byte attempts: 4;     // transmission attempts
byte count: 4;        // packet count
byte voltage;
byte salusAddress;
byte salusCommand;
byte salusNoise;
unsigned int ColdFeed;
unsigned int BoilerFeed;
unsigned int CentralHeatingReturn;
unsigned int TankCoilReturn;
//unsigned int HotFeed;
} payload;                                                                         //
/////////////////////////////////////////////////////////////////////////////////////

// Ten off wired DS18B20
// Blk=GND, Blue=DQ Red=Vdd
// 28 7F C6 4D 4 00 00 FF Tank Coil
// 28 7F CA 4D 4 00 00 DE Central Heating Return
// 28 53 4F 4E 4 00 00 84 Cold Feed
// 28 E8 AC 4E 4 00 00 EF
// 28 26 E6 4D 4 00 00 BF
// 28 86 39 4E 4 00 00 5A Boiler Feed
// 28 A0 68 4E 4 00 00 06
// 28 4E D3 4D 4 00 00 8B
// 28 F3 D1 4D 4 00 00 45
// 28 C9 C5 4D 4 00 00 04
//////////////////////////

#define ACK_TIME        49  // number of milliseconds - to wait for an ack, an initial 50ms
#define RETRY_LIMIT      7
byte NodeID = 16;
word lastCRC;

static byte sendACK() {
  for (byte t = 1; t <= RETRY_LIMIT; t++) {  
      delay(t * t);                   // Increasing the gap between retransmissions
      payload.attempts = t;
      rf12_sleep(RF12_WAKEUP);
      if(rf12_recvDone()) {
          // Serial.print("Discarded: ");                // Flush the buffer
          for (byte i = 0; i < 8; i++) {;
              showByte(rf12_buf[i]);
              rf12_buf[i] = 0xFF;                     // Paint it over
              printOneChar(' ');
          }
          // Serial.println();
          // Serial.flush(); 
      }
      rf12_sendStart(RF12_HDR_ACK, &payload, sizeof payload);
      byte acked = waitForAck(t * t); // Wait for increasingly longer time for the ACK
      if (acked) {
          for (byte i = 0; i < 6; i++) {
              showByte(rf12_buf[i]);
              printOneChar(' ');
          }
          // Serial.println();
          if(rf12_buf[2]) {
              payload.command = rf12_buf[3];
              // Serial.print(rf12_buf[3]);
              switch(rf12_buf[3]) {
                  case 1:
                      WatchSALUS = true;
                      payload.salusAddress = ~0;          // Salus monitoring restarted
                      payload.salusCommand = 0;           // ditto
                      // Serial.println("Salus Monitoring On");
                      break;
                  case 2:
                      WatchSALUS = false;
                      payload.salusAddress = 0;           // Salus monitoring suspended
                      payload.salusCommand = 0;           // ditto
                      // Serial.println("Salus Monitoring Off");
                      break;
                  case 140:
                      maxBoiler = rf12_buf[4] * 100;
                      // Serial.println("Set Boiler Feed Threshold");
                      break;      
                  case 150:
                      maxCH = rf12_buf[4] * 100;
                      // Serial.println("Set Central Heating Return Threshold");
                      break;      
                  default:
                      // Serial.println("Unknown Command");
//                      payload.command = 0;
                      break;      
                  }
          } // else payload.command = 0;
          return t;
      }
  }
  return 0;
} // sendACK

static byte waitForAck(byte t) {
    MilliTimer ackTimer;
    while (!ackTimer.poll(ACK_TIME + t)) {
        if (rf12_recvDone()) {
            rf12_sleep(RF12_SLEEP);

            // Serial.print((ACK_TIME + t) - ackTimer.remaining());
            // Serial.print("ms RX");
            if(rf12_crc == 0 &&
              // see http://talk.jeelabs.net/topic/811#post-4712
              rf12_hdr == (RF12_HDR_DST | RF12_HDR_CTL | NodeID)) {
                  // Serial.print(" ACK ");
                } else {
                    // Serial.print("Noise: ");                // Flush the buffer
                    for (byte i = 0; i < 8; i++) {;
                        showByte(rf12_buf[i]);
                        rf12_buf[i] = 0xFF;                 // Paint it over
                        printOneChar(' ');
                    }
                    showStats();                                
                }
                return 1;
        } 
        set_sleep_mode(SLEEP_MODE_IDLE);   // Wait a while for the reply?
        sleep_mode();
    }
    // Serial.print(ACK_TIME + t);
    // Serial.println("ms ACK Timeout");
    // Serial.flush();
    return 0;
} // waitForAck

static void printOneChar (char c) {
    // Serial.print(c);
}

static void showNibble (byte nibble) {
    char c = '0' + (nibble & 0x0F);
    if (c > '9')
        c += 7;
    // Serial.print(c);
}

static void showByte (byte value) {
//    if (config.output & 0x1) {
        showNibble(value >> 4);
        showNibble(value);
//    } else
//        // Serial.print((word) value, DEC);
}
static void showWord (unsigned int value) {
//    if (config.output & 0x1) {
        showByte (value >> 8);
        showByte (value);
//    } else
//        // Serial.print((word) value);    
}

unsigned int readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif  
 
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring
 
  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both
 
  unsigned int result = (high<<8) | low;
 
  result = ((1125300L / result)/100);   // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}

static word calcCrc (const void* ptr, byte len) {
    word crc = ~0;
    for (byte i = 0; i < len; ++i)
        crc = _crc16_update(crc, ((const byte*) ptr)[i]);
    return crc;
}

void setup () {
  delay(100);          // Delay on startup to avoid ISP/RFM12B interference.
  // Serial.begin(57600);
  // Serial.print((__DATE__));
  // Serial.print(" ");
  // Serial.println((__TIME__));
#if RF69_COMPAT
  // Serial.print("RFM69x ");
#else
  // Serial.print("RFM12x ");
#endif
  // Serial.print(SALUSFREQUENCY);  
  // Serial.print(" Heating monitor:");
  rf12_configDump();
  // Serial.flush();
  payload.BoilerFeed = ~0;
  payload.salusAddress = ~0;          // Until we know better
  payload.salusCommand = 0;           // ditto
  salusMillis = millis() + salusTimeout;

  pinMode(17, OUTPUT);      // Set the pin, AIO4 - Power the DS18B20's
  digitalWrite(17, HIGH);   // Power up the DS18B20's
  Sleepy::loseSomeTime((100 + 16));

/// Configure the DS18B20 ///
  ds.reset();               // Set for 12 bit measurements //
  ds.skip();                // Next command to all devices
  
  ds.write(0x4E);           // Write to Scratch Pad
  ds.write(0x7F);           // Set T(h)
  ds.write(0x80);           // Set T(l)
  ds.write(0x7F);           // Set Config 12 bit

/*  
/// Copy config to on-chip EEProm
  ds.reset();
  ds.skip();                // Next command to all devices
  ds.write(0x48);           // Set Config
*/
  Sleepy::loseSomeTime((100 + 16));// Wait for copy to complete
  digitalWrite(17, LOW);    // Power down the DS18B20's    

  } //  Setup
  
static void showStats() {
#if RF69_COMPAT
            // Serial.print(" a=");
            // Serial.print(RF69::afc);                        // TODO What units is this count?
            // Serial.print(" f=");
            // Serial.print(RF69::fei);                        // TODO What units is this count?
            // Serial.print(" l=");
            // Serial.print(RF69::lna >> 3);
            // Serial.print(" t=");
            // Serial.print((RF69::readTemperature(-10)));        
            // Serial.print(" (");
            // Serial.print(RF69::rssi >> 1);
            if (RF69::rssi & 0x01) // Serial.print(".5");
            // Serial.print("dB)");
#endif
            // Serial.println();
            // Serial.flush();
}

unsigned int getTemp(byte* sensor) {
  byte i;
  byte present = 0;
  byte data[12]; 

  ds.reset();
  ds.select(sensor);    
  ds.write(0xBE);                                            // Read Scratchpad

  // Serial.print("Data = ");
//  // Serial.print(present,HEX);
//  // Serial.print(" ");

  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
    // Serial.print(data[i], HEX);
    // Serial.print(" ");
  }
/*
  // Serial.print(" CRC=");
  // Serial.print(OneWire::crc8(data, 8), HEX);
  // Serial.println();
*/
  // convert the data to actual temperature

  long int raw = (data[1] << 8) | data[0];
// Assuming 12 bit precision
  raw = raw * 100;
  raw = raw >> 4;
  return (raw); // return t*100
}

void loop () {
/*
 * Setup to receive Salus transmissions
 */
        rf12_initialize (SALUSID, RF12_868MHZ, 212, SALUSFREQUENCY);  // 868.360khz
        rf12_sleep(RF12_SLEEP);                                       // Sleep while we tweak things
#if RF69_COMPAT
        RF69::control(REG_BITRATEMSB | 0x80, 0x34);                   // 2.4kbps
        RF69::control(REG_BITRATELSB | 0x80, 0x15);
        RF69::control(REG_BITFDEVMSB | 0x80, 0x03);                   // 60kHz freq shift
        RF69::control(REG_BITFDEVLSB | 0x80, 0xD7);
        rf12_sleep(RF12_WAKEUP);                                      // All set, wake up radio
#else
        rf12_control(0xC040);                                         // set low-battery level to 2.2V
        rf12_control(RF12_DATA_RATE_2);                               // 0xC691 app 2.4kbps
        rf12_control(0x9830);                                         // 60khz freq shift
#endif

        if (((millis() > salusMillis) || needOff)) {                  // Is a Salus Off required?
            needOff = false;
            salusMillis = millis() + salusTimeout;
            // Serial.println("Sending Salus off");
            rf12_sleep(RF12_WAKEUP);
//            while (!rf12_canSend())
            rf12_skip_hdr();                                          // Omit Jeelib header 2 bytes on transmission
            rf12_sendStart(0, &salusOff, 4);                          // Transmit the Salus off command
            rf12_sendWait(1);                                         // Wait for transmission complete
            rf12_sleep(RF12_SLEEP);
            payload.salusAddress = 0;                                 // Note the prolonged loss of contact
            payload.salusCommand = OFF | 0x80;                        // Update the Jee world status
        }                                                             // 0x80 indicates Jeenode commanded off
                
    if (WatchSALUS) {        
        rf12_sleep(RF12_WAKEUP);                                      // Wake up radio
        rf12_recvDone();                                              // Enter receive mode
        // Serial.println("Waiting for Salus");
        // Serial.flush();
    
        elapsed = elapsed + (TEMPCHECK - (Sleepy::idleSomeTime(TEMPCHECK)));      // Check temperatures every minute

#if !RF69_COMPAT
        delay(10);                                                            // Wait for the data to be available
        if (rf12_buf[4] == 90)
#else
        if (rf12_recvDone())
#endif
            {
            if ((rf12_buf[0] == 212 && ((rf12_buf[1] | rf12_buf[2]) == rf12_buf[3]) 
              && rf12_buf[4] == 90/* && rf12_buf[1] == SALUSID*/)) {      // Capture all Salus packets
                rf12_buf[4] = ~90;
                rf12_sleep(RF12_SLEEP);
                salusMillis = millis() + salusTimeout;
                // Serial.print("Salus: ");
                // Serial.print(rf12_buf[1]);
                // Serial.print(",");
                // Serial.print(rf12_buf[2]);
                showStats();
                // Serial.flush();
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
                // Serial.println();
            }
        }
    } else  {
      rf12_sleep(RF12_SLEEP);                                                   // Sleep the radio
      elapsed = elapsed + (TEMPCHECK - (Sleepy::idleSomeTime(TEMPCHECK)));      // Check temperatures every minute
    }

    if (elapsed >= TEMPCHECK) {
        digitalWrite(17, HIGH);                                   // Power up the DS18B20's
        Sleepy::loseSomeTime((10 + 16)); 
        ds.reset();
        ds.skip();                                                // Next command to all devices
        ds.write(0x44);                                           // Start all temperature conversions.
        Sleepy::loseSomeTime((750 + 16));                               // Wait for the data to be available

        payload.ColdFeed = getTemp(ColdFeed);
        // Serial.print("Cold Feed:");
        // Serial.println(payload.ColdFeed);

        payload.BoilerFeed = getTemp(BoilerFeed);
        // Serial.print("Boiler Feed:");
        // Serial.println(payload.BoilerFeed);

        payload.CentralHeatingReturn = (getTemp(CentralHeatingReturn)) - 12;
        // Serial.print("Heating Return:");
        // Serial.println(payload.CentralHeatingReturn);

        payload.TankCoilReturn = getTemp(TankCoilReturn);
        // Serial.print("Coil Return:");
        // Serial.println(payload.TankCoilReturn);

        digitalWrite(17, LOW);                                     // Power down the DS18B20's    
        // Serial.flush();

        word thisCRC = calcCrc(&payload.voltage, sizeof payload - 1);
        if (thisCRC != lastCRC) {
            // Serial.println(thisCRC);
            lastCRC = thisCRC;
            payload.count++;
            if (NodeID = rf12_configSilent()) {
                // Serial.print("Node ");
                // Serial.print(NodeID);
                // Serial.print(" sending packet #");
                // Serial.println(payload.count);
                // Serial.flush();
            
                byte tries = sendACK();
            
                if(tries) { 
                    // Serial.print(tries);
                    // Serial.println(" attempt(s)");
                } else {
                    // Serial.println("Aborted");
                }
                
                if (payload.BoilerFeed >= maxBoiler) {
                    // Serial.println("Boiler feed above threshold");
                    needOff = true;
                }

                if (payload.CentralHeatingReturn >= maxCH) {
                    // Serial.println("C/H Return above threshold");
                    needOff = true;
                }
               // Serial.flush();             
            } else {
                  while( true ){
                    rf12_sleep(RF12_SLEEP);
                    // Serial.println("RF12 eeprom not valid, run RF12Demo");
                    // Serial.flush();
                    Sleepy::idleSomeTime(60);
                  }  
            }
            elapsed = 0; 
        }
    } else {
          // Serial.print(elapsed);
          // Serial.println(" seconds elapsed");
    }

    // Serial.print("Voltage:");
    payload.voltage = readVcc();
    // Serial.println(payload.voltage);
    if (payload.voltage > 28) {
        // Serial.print("Looping ");
        // Serial.println(++loopCount);
        // Serial.flush();
    } else {
        rf12_sleep(RF12_SLEEP);
        // Serial.println("Replace batteries");
        // Serial.flush();
        cli();
        Sleepy::powerDown();
    }
} // Loop

