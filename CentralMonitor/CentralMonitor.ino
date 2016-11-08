///////////////////////////////////////////////////////////////////////////////
#define RF69_COMPAT      0   // define this to use the RF69 driver i.s.o. RF12 
//                           // The above flag must be set similarly in RF12.cpp
//                           // and RF69_avr.h
///////////////////////////////////////////////////////////////////////////////
// 
#include <JeeLib.h>
#include <avr/sleep.h>
#include <OneWire.h>
#include <avr/eeprom.h>
#include <util/crc16.h>

#define crc_update      _crc16_update
#define AIO2 9 // d9
#define SALUSID 16
#define SALUSFREQUENCY 1660
#define ON  1
#define OFF 2
#define TEMPCHECK 9        // Check the temperatures every 9 seconds

#define REG_BITRATEMSB 0x03 // RFM69 only, 0x02, // BitRateMsb, data rate = 49,261 khz
#define REG_BITRATELSB 0x04 // RFM69 only, 0x8A, // BitRateLsb divider = 32 MHz / 650 == 49,230 khz
#define REG_BITFDEVMSB 0x05
#define REG_BITFDEVLSB 0x06

#define SETTINGS_EEPROM_ADDR ((uint8_t*) 0x00)

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
 * typically SETBACK commands to optimise heating performance and gas consumption.
 * 
 * In addition to the above various temperatures around the heating system 
 * will be collected as below. Where the central heating return approaches 
 * the boiler feed temperature a Salus command will be issued to cancel
 * the central heating demand by issuing a SETBACK to the boiler controller. 
 * This temperature difference is where the tuning of gas consumption may be acheived. 
 * It should be noted that the Salus thermostat will repeat its instructions 
 * to the boiler based on temperature and every 10 minutes. The SETBACK command must 
 * also be refreshed every 10 minutes to maintain boiler status. If this sketch
 * cancels the heating demand the demand will be reasserted by the thermostat if 
 * conditions dictate.
 * 
 * <review> If the Salus thermostat fails to deliver its demand status on its regular
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
byte commandOTO[] = {166, 163, 106, 0, 179};
//byte setBack0[] = {166, 163, 106, 0, 179};
//byte setBack2[] = {166, 163, 106, 4, 183}; // Setback temperature by 2 degrees
byte setBack0[] = {166, 163, 106, 24, 203};
byte setBack1[] = {166, 163, 106, 28, 207}; // Setback temperature by AUTO degrees
byte setback = false; byte needSetback = false;
byte backCount = 0;
unsigned int elapsed = 0;
int previousBoilerFeed;
int previousReturn;
ISR(WDT_vect) { Sleepy::watchdogEvent(); }
//
/////////////////////////////////////////////////////////////////////////////////////
struct payload{                                                                    //
byte command;         // Last command received in ACK
byte badCRC:  4;      // Running count of CRC mismatches
byte setBack: 2;      // True if a setback is pending
byte packetType:  2;  // High order packet type bits
byte attempts: 4;     // transmission attempts
byte count: 4;        // packet count
byte voltage;
unsigned int salusAddress;
byte salusCommand;
byte salusNoise;
unsigned int currentTemp;
unsigned int lowestTemp;
unsigned int targetTemp;
unsigned int ColdFeed;
unsigned int BoilerFeed;
unsigned int CentralHeatingReturn;
unsigned int TankCoilReturn;
unsigned int boiler_target;
unsigned int overRun;   // Temperature overrun
unsigned int underRun;  // Temperature underrun
unsigned int onTarget;  // Temperature correct
#define BASIC_PAYLOAD_SIZE 30
char messages[64 - BASIC_PAYLOAD_SIZE];
} payload;
//
/////////////////////////////////////////////////////////////////////////////////////
typedef struct {
    byte start;
    byte WatchSALUS:1;
    byte spare:7;
    unsigned int maxBoiler;
    unsigned int maxReturn;
    byte txSkip;
    byte spare2;	// Unused
    unsigned int addrOTO;
    byte checkOTO;
    word crc;
} eeprom;
static eeprom settings;
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

#define ACK_TIME        100  // number of milliseconds - to wait for an ack, an initial 100ms
#define RETRY_LIMIT      7

byte payloadSize = BASIC_PAYLOAD_SIZE;
byte NodeID = 16;
word lastCRC;
byte txSkip = 1;
byte commandResponse = false;
byte getOTO = false;
signed int boilerTrend;
signed int returnTrend;

static byte sendACK() {
  payload.boiler_target = settings.maxBoiler;
  
  for (byte t = 1; t <= RETRY_LIMIT; t++) {  
      delay(t * t);                   // Increasing the gap between retransmissions
      payload.attempts = t;
      rf12_sleep(RF12_WAKEUP);
      if (rf12_recvDone()) {
          // Serial.print("Discarded: ");             // Flush the buffer
          for (byte i = 0; i < 8; i++) {
              showByte(rf12_buf[i]);
              rf12_buf[i] = 0xFF;                     // Paint it over
              printOneChar(' ');
          }
          // Serial.println();
          // Serial.flush(); 
      }
      
      while (!(rf12_canSend())) {
        // Serial.print("Airwaves Busy");
        Sleepy::loseSomeTime((50)); 
      }
        
      // Serial.println("TX Start");
      rf12_sendStart(RF12_HDR_ACK, &payload, payloadSize);
      // Serial.println("TX Done");
      // Serial.flush();      
      byte acked = waitForAck(t * t); // Wait for increasingly longer time for the ACK
      if (acked) {
          payloadSize = BASIC_PAYLOAD_SIZE;   // Packet was ACK'ed by someone
          commandResponse = false;
          payload.packetType = 0;                         
          for (byte i = 0; i < 6; i++) {
              showByte(rf12_buf[i]);
              printOneChar(' ');
          }
          // Serial.println();
          if (rf12_buf[2] > 0) {                          // Non-zero length ACK packet?
              payload.command = rf12_buf[3];
              // Serial.print("Command=");
              // Serial.println(rf12_buf[3]);
              commandResponse = true;

              if ((rf12_len + 5) > sizeof payload.messages) rf12_len = (sizeof payload.messages - 5); 
              for (byte i = 0; i < (rf12_len + 5); i++) {
                  payload.messages [i] = rf12_buf[i];     // Return command stream with next packet
                  payloadSize = BASIC_PAYLOAD_SIZE + (rf12_len + 5);
              }
              
              switch (rf12_buf[3]) {
                  case 1:
                      payload.overRun = payload.underRun = payload.onTarget = 0;                      
                      // Serial.println("Underrun/Overrun/On target cleared");
                      break;
                  case 2:
                  		settings.txSkip = 2;
                  		// Serial.println("TX Skip 2");
                  		break;
                  case 6:
                  		settings.txSkip = 6;
                  		// Serial.println("TX Skip 6");
                  		break;
                  case 12:
                  		settings.txSkip = 12;
                  		// Serial.println("TX Skip 12");
                  		break;
                  case 30:
                  		settings.txSkip = 30;
                  		// Serial.println("TX Skip 30");
                  		break;
                  case 89:  // Capture OTO code
                      getOTO = true;
                      // Serial.println("Learning OTO codes");
                      break;
                  case 99:
                      // Serial.println("Saving settings to eeprom");
                      saveSettings();
                      break;      
                  default:
                      if (rf12_buf[3] > 100 && rf12_buf[3] < 200) {
                          settings.maxBoiler = (rf12_buf[3] - 100) * 100;
                          // Serial.print("Setting Boiler Feed Threshold:");
                          // Serial.println(settings.maxBoiler);
                          break;  
                      }
                      if (rf12_buf[3] > 200 && rf12_buf[3] < 255) {
                          settings.maxReturn = (rf12_buf[3] - 200) * 100;
                          // Serial.print("Setting Central Heating Return Threshold:");
                          // Serial.println(settings.maxReturn);
                          break;
                      }
                      // Serial.println("Unknown Command");
                      break;

                  } // end switch
          }
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
            // Serial.print("ms RX ");
            
            if (rf12_crc == 0) {                          // Valid packet?
                // see http://talk.jeelabs.net/topic/811#post-4712
                if (rf12_hdr == (RF12_HDR_DST | RF12_HDR_CTL | NodeID)) {
                    // Serial.print("ACK ");
                    return 1;            
                } else {
                    // Serial.print("Unmatched: ");             // Flush the buffer
                    for (byte i = 0; i < 8; i++) {;
                        showByte(rf12_buf[i]);
                        rf12_buf[i] = 0xFF;              // Paint it over
                        printOneChar(' ');
                    }
                    showStats();                                
                }
            } else {
                // Serial.print("Bad CRC");
                payload.badCRC++;
            }
            // Serial.println(); // Serial.flush();           
        } 
        set_sleep_mode(SLEEP_MODE_IDLE);   // Wait a while for the reply?
        sleep_mode();
    }
//    printOneChar(' ');
    // Serial.print(ACK_TIME + t);
    // Serial.println("ms ACK Timeout");
    // Serial.flush();
    return 0;
} // waitForAck

static void saveSettings () {
    settings.start = ~0;
    settings.crc = calcCrc(&settings, sizeof settings - 2);
    // this uses 170 bytes less flash than eeprom_write_block(), no idea why
    for (byte i = 0; i < sizeof settings; ++i) {
        byte* p = &settings.start;
        if (eeprom_read_byte(SETTINGS_EEPROM_ADDR + i) != p[i]) {
            eeprom_write_byte(SETTINGS_EEPROM_ADDR + i, p[i]);
        }
    }
} // saveSettings

static void loadSettings () {
    uint16_t crc = ~0;
    // eeprom_read_block(&settings, SETTINGS_EEPROM_ADDR, sizeof settings);
    // this uses 166 bytes less flash than eeprom_read_block(), no idea why
    for (byte i = 0; i < sizeof settings; ++i) {
        ((byte*) &settings)[i] = eeprom_read_byte(SETTINGS_EEPROM_ADDR + i);
        crc = crc_update(crc, ((byte*) &settings)[i]);
    }
    // Serial.print("Settings CRC ");
    if (crc) {
        // Serial.println("is bad, defaulting");
        // Serial.println(crc, HEX);
        settings.WatchSALUS = true;
        settings.maxBoiler = 4500;
        settings.maxReturn = 6500;
        settings.txSkip = 6;
    } else {
        // Serial.println("is good");
    }
    // Serial.print("Boiler threshold:");
    // Serial.println(settings.maxBoiler);
    // Serial.print("C/H return threshold:");
    // Serial.println(settings.maxReturn);
} // loadSettings

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

void checkSetBack () {
        rf12_sleep(RF12_WAKEUP);                                      // All set, wake up radio
        backCount++;
        if (!needSetback) {
            if (setback) {
                rf12_sendStart(0, &setBack0, 5);                          // Issue a OTO to cancel setback.
                rf12_sendWait(1);                                         // Wait for transmission complete.
                // Serial.println("Setback Cancelled");
                rf12_sendStart(0, &setBack0, 5);                          // Issue a OTO to cancel setback.
                rf12_sendWait(1);                                         // Wait for transmission complete.
                backCount = 0;                                            // OTO every 10 mins
                setback = false;
            }
        } else {
            if (!setback) {
                rf12_sendStart(0, &setBack1, 5);                          // Issue a OTO to setback.
                rf12_sendWait(1);                                         // Wait for transmission complete.
                // Serial.println("Setback Issued");
                rf12_sendStart(0, &setBack1, 5);                          // Issue a OTO to setback.
                rf12_sendWait(1);                                         // Wait for transmission complete.
                backCount = 0;                                            // OTO every 10 mins
                setback = true;
            }
        }
        if (backCount > 9) {
            if (setback) {
                rf12_sendStart(0, &setBack1, 5);                          // Issue a OTO to refresh setback.
                payload.setBack = 1;
            } else {
                rf12_sendStart(0, &setBack0, 5);                          // Issue a OTO to refresh null setback.
                payload.setBack = 0;
            }
            rf12_sendWait(1);                                             // Wait for transmission complete.
            // Serial.println("Setback Refreshed");
            backCount = 0;                                                // OTO every 10 mins
        }
        // Serial.flush();
        rf12_sleep(RF12_SLEEP);
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
  loadSettings();
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
/// Copy config to on-chip EEProm, only required on first use of a new DS18B20
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
            return;
}

unsigned int getTemp(byte* sensor) {
  byte i;
  byte present = 0;
  byte data[12]; 

  ds.reset();
  ds.select(sensor);    
  ds.write(0xBE);                                            // Request Scratchpad

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
        rf12_skip_hdr(2);                                             // Omit Jeelib header 2 bytes on transmission
        rf12_fix_len(15);                                             // Maximum fixed length packet size.
#if RF69_COMPAT
        RF69::control(REG_BITRATEMSB | 0x80, 0x34);                   // 2.4kbps
        RF69::control(REG_BITRATELSB | 0x80, 0x15);
        RF69::control(REG_BITFDEVMSB | 0x80, 0x03);                   // 60kHz freq shift
        RF69::control(REG_BITFDEVLSB | 0x80, 0xD7);
#else
        rf12_control(0xC040);                                         // set low-battery level to 2.2V
        rf12_control(RF12_DATA_RATE_2);                               // 0xC691 app 2.4kbps
        rf12_control(0x9830);                                         // 60khz freq shift
#endif
        checkSetBack();
/*
        if (((millis() > salusMillis) || needOff)) {                  // Is a Salus Off required?
            needOff = false;
            salusMillis = millis() + salusTimeout;
            // Serial.println("Sending Salus off");
            rf12_sleep(RF12_WAKEUP);
            rf12_sendStart(0, &salusOff, 4);                          // Transmit the Salus off command
            rf12_sendWait(1);                                         // Wait for transmission complete
            rf12_sleep(RF12_SLEEP);
            payload.salusAddress = 0;                                 // Note the prolonged loss of contact
            payload.salusCommand = OFF | 0xF0;                        // Update the Jee world status
        }                                                             // 0x80 indicates Jeenode commanded off
*/        
        for (byte i = 0; i < 66; i++) rf12_buf[i] = 0;                // Clear buffer
    
    if (settings.WatchSALUS) {        
        rf12_sleep(RF12_WAKEUP);                                      // Wake up radio
        rf12_recvDone();                                              // Enter receive mode
        // Serial.println("Waiting for Salus");
        // Serial.flush();
    
        if (!commandResponse) elapsed = elapsed + (TEMPCHECK - (Sleepy::idleSomeTime(TEMPCHECK)));  
        // Check temperatures every minute

#if !RF69_COMPAT
        delay(120);                                                             // Wait for the full packet to be available
#endif
        if (rf12_recvDone() && rf12_buf[0] == 212)
            {
                rf12_sleep(RF12_SLEEP);

                for (byte i = 0; i < 13; i++) {
                    showByte(rf12_buf[i]);
                    printOneChar(' ');
                }
                // Serial.println();
                // Serial.flush();

              
            if ((rf12_buf[0] == 212) && ((rf12_buf[1] | rf12_buf[2]) == rf12_buf[3])) { 
//              && rf12_buf[4] == 90/* && rf12_buf[1] == SALUSID*/))       // Capture all Salus packets
//                rf12_buf[4] = ~90;
                salusMillis = millis() + salusTimeout;
                // Serial.print("Salus 1 ");
                // Serial.print(rf12_buf[1]);
                // Serial.print(",");
                // Serial.print(rf12_buf[2]);
                showStats();
                // Serial.flush();
                elapsed = ~0;                                             // Trigger a transmit
                payload.salusAddress = rf12_buf[1];   
                payload.salusCommand = rf12_buf[2];   
                payload.packetType |= 1;        // Indicate new data
                Sleepy::loseSomeTime(50 * 29);  // Salus sends 30 packets spaced at 50ms so wait for the redundant packets to pass us by
            } else if (rf12_buf[0] == 212 && rf12_buf[1] >= 160) {
                // Serial.print("Salus II Device:"); // Serial.flush();
                Sleepy::loseSomeTime(150 * 8);  // Salus sends 8 packets spaced at 150ms so wait for the redundant packets to pass us by
                // Serial.print(rf12_buf[1]);  // Device type

                // Serial.print(" Addr:");
                payload.salusAddress = (rf12_buf[3] << 8) | rf12_buf[2];   // Guessing at a 16 bit address
                // Serial.print(payload.salusAddress);
                // Serial.print(" Command:");
                payload.salusCommand = rf12_buf[4];
                // Serial.print(payload.salusCommand);
                unsigned int crc;
                switch (rf12_buf[1]) {
                    case 165: // Thermostat
                        payload.currentTemp = ((rf12_buf[6] << 8) | rf12_buf[5]);
                        // Serial.print(" Current="); // Serial.print(payload.currentTemp);
                        payload.lowestTemp = ((rf12_buf[8] << 8) | rf12_buf[7]);
                        // Serial.print(" Lowest=");  // Serial.print(payload.lowestTemp);
                        payload.targetTemp = ((rf12_buf[10] << 8) | rf12_buf[9]);
                        // Serial.print(" Target=");  // Serial.print(payload.targetTemp);
                        payload.packetType |= 2;      // Indicate new data
                        
                        if (payload.lowestTemp != payload.targetTemp) { // Typical daytime setting
                            if (payload.currentTemp < payload.targetTemp) payload.underRun++;
                            if (payload.currentTemp > payload.targetTemp) payload.overRun++;
                            if (payload.currentTemp == payload.targetTemp) payload.onTarget++;
                        }
                        txSkip = ~0;	// Trigger a transmission
//                        crc = ((rf12_buf[12] << 8) | rf12_buf[11]);
                        // Serial.print(" crc=0x"); // Serial.print(crc, HEX);
/*                        if (payload.salusCommand == 32) {
                            setback = false;
                            checkSetBack();
                        }  */                      
                        elapsed = ~0;                                             // Trigger a transmit
                        break;
                    case 166:   // OTO One Touch Override
                        printOneChar(' ');
                        // Serial.print(rf12_buf[5]);
                        if (getOTO) {
                            settings.addrOTO = payload.salusAddress;
                            settings.checkOTO = (rf12_buf[5] - rf12_buf[4]);
                            // Serial.print(" Learned OTO Offset "); // Serial.println(settings.checkOTO); // Serial.flush();
                            getOTO = false; // Learn no more
                        } else if ((payload.salusAddress == settings.addrOTO) && ((rf12_buf[5]) - rf12_buf[4] == settings.checkOTO)) {
                            // Serial.print(" One Touch Override Matched");
                            // Serial.println(", Relaying OTO"); // Serial.flush();
                            // Update checksum // byte commandOTO[] = {166, 163, 106, 0, 179};
                            commandOTO[4] = (commandOTO[4] - commandOTO[3]) + rf12_buf[4];
                            commandOTO[3] = rf12_buf[4];              // Copy over command byte
                            //
                            // Serial.print(commandOTO[3]); printOneChar(':'); // Serial.print(commandOTO[4]); // Serial.flush();
                            rf12_sleep(RF12_WAKEUP);
                            rf12_sendStart(0, &commandOTO, 5);               // Forward the OTO command.
                            rf12_sendWait(1);                                         // Wait for transmission complete.
                            rf12_sleep(RF12_SLEEP);                                
                        } else // Serial.println(" One Touch Override Unknown"); // Serial.flush();
                        break;
                    default:
                        // Serial.print(" Unknown ");
                        for (byte i = 0; i < 8; i++) {;
                            showByte(rf12_buf[i]);
                            printOneChar(' ');
                        }
                        // Serial.flush();
                        break;
                }
                showStats();
                // Serial.println(); // Serial.flush();
                
            } else {  
                // Serial.print("Non Salus Noise ");
                payload.salusNoise++;
                for (byte i = 0; i < 8; i++) {;
                    showByte(rf12_buf[i]);
                    printOneChar(' ');
                }
                // Serial.println(); // Serial.flush();
            }
        }
    } else  {
      // Serial.flush();
      rf12_sleep(RF12_SLEEP);                                                   // Sleep the radio
      if (!commandResponse) elapsed = elapsed + (TEMPCHECK - (Sleepy::idleSomeTime(TEMPCHECK)));      
    }

    if ((elapsed >= TEMPCHECK) || (commandResponse)) {
        digitalWrite(17, HIGH);                                   // Power up the DS18B20's
        Sleepy::loseSomeTime((50)); 
        ds.reset();
        ds.skip();                                                // Next command to all devices
        ds.write(0x44);                                           // Start all temperature conversions.
        Sleepy::loseSomeTime((750));                              // Wait for the data to be available

        payload.ColdFeed = getTemp(ColdFeed);
        // Serial.print("Cold Feed:");
        // Serial.println(payload.ColdFeed);

        payload.BoilerFeed = getTemp(BoilerFeed);
        boilerTrend = 0;
        if (previousBoilerFeed) boilerTrend = payload.BoilerFeed - previousBoilerFeed;
        previousBoilerFeed = payload.BoilerFeed;
        // Serial.print("Boiler Feed:");
        // Serial.print(payload.BoilerFeed);
        // Serial.print(" trend:");
        // Serial.println(boilerTrend);
                        
        payload.CentralHeatingReturn = (getTemp(CentralHeatingReturn)) - 12;
        returnTrend = 0;
        if (previousReturn) returnTrend = payload.CentralHeatingReturn - previousReturn;
        previousReturn = payload.CentralHeatingReturn;
        // Serial.print("Heating Return:");
        // Serial.print(payload.CentralHeatingReturn);
        // Serial.print(" trend:");
        // Serial.println(returnTrend);
        
        payload.TankCoilReturn = getTemp(TankCoilReturn);
        // Serial.print("Coil Return:");
        // Serial.println(payload.TankCoilReturn);

        digitalWrite(17, LOW);									// Power down the DS18B20    

        if (boilerTrend > 0) {
        	if ((payload.currentTemp + 50) >= payload.targetTemp) {          
                // Serial.println("More than half a degree under target");
            	if (payload.BoilerFeed > settings.maxBoiler) {	// Heats up much faster            
                	// Serial.println("Above threshold");
                	if (!(needSetback)) txSkip = ~0;
                	needSetback = true;
                	payload.setBack = 1;
                }
            }
        } else 
        	if ((payload.BoilerFeed < settings.maxBoiler) || (payload.currentTemp + 50) 
        	  < payload.targetTemp) {								// than it cools down!             
        	// Serial.println("Below threshold or more than half a degree under target");
            if (needSetback) txSkip = ~0;
        	needSetback = false;
        	payload.setBack = 0; 
        }
        // Serial.flush();

        if (txSkip > settings.txSkip) {
            txSkip = 0;
            payload.count++;
            if (NodeID = rf12_configSilent()) {
                // Serial.print("Node ");
                // Serial.print(NodeID);
                // Serial.print(" sending packet #");
                // Serial.print(payload.count);
                // Serial.print(" length ");
                // Serial.println(payloadSize);
                // Serial.flush();
            
                Sleepy::loseSomeTime(1000); // Allow time for Jeelink receiver to recover from Salus packets.
                byte tries = sendACK();
            
                if (tries) { 
                    // Serial.print(tries);
                    // Serial.println(" attempt(s)");
                } else {
                    // Serial.print("Packet #");
                    // Serial.print(payload.count);
                    // Serial.println(" Aborted");
                }
               // Serial.flush();             
            } else {
                  while( true ){
                    rf12_sleep(RF12_SLEEP);
                    // Serial.println("RF12 eeprom not valid, run RF12Demo"); // Serial.flush();
                    Sleepy::idleSomeTime(60);
                  }  
            }
            elapsed = 0; 
        } else {
          txSkip++;
          // Serial.print("Skipped transmission #");
          // Serial.println(txSkip);
          // Serial.flush();
        }
    } else {
          // Serial.print(elapsed);
          // Serial.println(" seconds elapsed");
          // Serial.flush();
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

