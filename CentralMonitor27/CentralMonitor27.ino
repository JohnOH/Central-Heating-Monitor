///////////////////////////////////////////////////////////////////////////////
#define RF69_COMPAT      0   // define this to use the RF69 driver i.s.o. RF12
//                           // The above flag must be set similarly in RF12.cpp
//                           // and RF69_avr.h
///////////////////////////////////////////////////////////////////////////////
//
#include <JeeLib.h>
#include "RFAPI.h"		// Define
rfAPI rfapi;			// Declare
#include <avr/sleep.h>
#include <OneWire.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <util/crc16.h>
#include <avr/wdt.h>

#define crc_update      _crc16_update
#define AIO2 9 // d9
#define SALUSID 16
#define SALUSFREQUENCY 1660
#define ON  1
#define OFF 2

#define REG_BITRATEMSB 0x03 // RFM69 only, 0x02, // BitRateMsb, data rate = 49,261 khz
#define REG_BITRATELSB 0x04 // RFM69 only, 0x8A, // BitRateLsb divider = 32 MHz / 650 == 49,230 khz
#define REG_BITFDEVMSB 0x05
#define REG_BITFDEVLSB 0x06

#define SETTINGS_EEPROM_ADDR ((uint8_t*) 0x00)

#define DEBUG 0			// If enabled OTO commands are NOT transmitted!

#if DEBUG
	#define D(x) x
#else
	#define D(x)
#endif
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
 * withholds the heating demand the demand will still be requested by the thermostat if
 * conditions dictate.  28 26 E6 4D 4 0 0 BF28 26 E6 4D 4 0 0 BF
 *
 * <review> If the Salus thermostat fails to deliver its demand status on its regular
 * schedule then this sketch will cancel the central heating demand.
 *
 * 13/04/2017 Tidy up lots of problems hearing Salus commands
 * 19/04/2020 Accomodate extended basic ACK
 */

unsigned long setbackMax = 595;	// Close to 10 minutes
unsigned long backCount = setbackMax;
unsigned long minute = 60;	// note the approx 2 seconds delay in code
volatile unsigned long seconds, elapsedSeconds, nextScheduled, setbackTimer;
unsigned long delaySeconds, waitSeconds;
unsigned int loopCount;

byte ColdFeed[8] = {0x28,0xA0,0x68,0x4E,0x04,0x00,0x00,0x06};
byte BoilerFeed[8] = {0x28,0x26,0xE6,0x4D,0x04,0x00,0x00,0xBF};
byte CentralHeatingReturn[8] = {0x28,0x4E,0xD3,0x4D,0x4,0x0,0x0,0x8B};
byte HWTankTemp[8] = {0x28,0xF3,0xD1,0x4D,0x04,0x00,0x00,0x45};

byte addr[8];
byte salusOff[] = {SALUSID, OFF, SALUSID | OFF, 90};
/*
    Commands: 00/04 is for 2 degrees setback switch
              08/12 is for 4 degrees setback switch
              16/20 is for 6 degrees setback switch
              24/28 is for Auto degrees setback switch

byte commandOTO[] = {166, 163, 106, 0, 179};
byte setBack0[] = {166, 163, 106, 24, 203};
byte setBack2[] = {166, 163, 106, 4, 183}; // Setback temperature by 2 degrees
byte setBack4[] = {166, 163, 106, 8, 183}; // Setback temperature by 4 degrees
byte setBack6[] = {166, 163, 106, 16, 183}; // Setback temperature by 6 degrees
byte setBackA[] = {166, 163, 106, 28, 207}; // Setback temperature by AUTO degrees
*/

/* https://calctools.online/en/checksum/sum

Add together the four bytes of data, 0xA6, 0x42, 0x24, 0x10
the lower 8 bites of the result is the checksum as the fifth byte.

*/
// OTO controller commands
byte commandOTO[] = {0xA6, 0x42, 0x24, 0, 0x0C}; // A6 42, 24 0
byte setBack0[] = {0xA6, 0x42, 0x24, 0x18, 0x24};	// Auto Release (24) // A6 42 24 18
byte setBack2[] = {0xA6, 0x42, 0x24, 0x04, 0x10}; // Setback temperature by 2 degrees (4) // A6 42 24 04
byte setBack4[] = {0xA6, 0x42, 0x24, 0x0C, 0x18}; // Setback temperature by 4 degrees (12) // A6 42 24 0C
byte setBack6[] = {0xA6, 0x42, 0x24, 0x14, 0x20}; // Setback temperature by 6 degrees (20) // A6 42 24 14
byte setBackA[] = {0xA6, 0x42, 0x24, 0x1C, 0x28}; // Setback temperature by AUTO degrees (28)
byte setPairing[] = {0xA6, 0x42, 0x24, 0x1A, 0x26}; // Pair with unit on cylinder?thermostat (8)
byte eight[] = {0xA6, 0x42, 0x24, 0x08, 0x14};		// Power on OTO (8)
byte sixteen[] = {0xA6, 0x42, 0x24, 0x10, 0x1C};	// OTO release setback 6 (16)

// Thermostat controller commands
byte setHeatingOn[] = {0xA5, 0x10, 0x00, 0x20, 0x08, 0x07, 0xF4, 0x01, 0x6C, 0x07, 0x32, 0x7E}; // Heating on

byte setback = false; byte needSetback = false;
int previousBoilerFeed;
int previousReturn;
unsigned int tenK = 10000;
//
byte previousKey;
/////////////////////////////////////////////////////////////////////////////////////
struct payload{                                                                    //
byte ackKey;		  // Last command received in ACK
byte badCRC:  4;      // Running count of CRC mismatches
byte tracking:	1;	  // True if we are tracking
byte setBack: 2;      // True if a setback is pending
byte packetType:  1;  // High order packet type bits
byte attempts: 4;     // transmission attempts
byte count: 4;        // packet count
byte tick;
unsigned int salusAddress;
byte salusCommand;
byte salusNoise;
unsigned int currentTemp;
unsigned int lowestTemp;
unsigned int targetTemp;
unsigned int ColdFeed;
unsigned int BoilerFeed;
unsigned int CentralHeatingReturn;
unsigned int HWTankTemp;
unsigned int boiler_target;
unsigned int overRun;   // Temperature overrun
unsigned int underRun;  // Temperature underrun
unsigned int onTarget;  // Temperature correct
unsigned int status;	// Debugging Flag
unsigned int elapsed;	// 16bits of elapsed seconds
#define BASIC_PAYLOAD_SIZE 34
byte messages[64 - BASIC_PAYLOAD_SIZE];
} payload;
//
/////////////////////////////////////////////////////////////////////////////////////
typedef struct {
    byte start;	// 0x55
    byte spare:6;
    byte WatchSALUS:1;
    byte tracking:1;
    unsigned int maxBoiler;
    unsigned int burnTime1;
    unsigned int burnTime2;
    byte burnTime3;
    unsigned int SalusAddress;
    byte salusTX;	// Transmit power when sending OTO commands
    unsigned int addrOTO;
    byte checkOTO;
    word crc;
} eeprom;
static eeprom settings;
/////////////////////////////////////////////////////////////////////////////////////

// Ten off wired DS18B20
// Blk=GND, Blue=DQ Red=Vdd
// 28 7F C6 4D 4 00 00 FF Hot Water Tank (17)
// 28 7F CA 4D 4 00 00 DE Central Heating Return (17)
// 28 53 4F 4E 4 00 00 84 Cold Feed (17)
// 28 E8 AC 4E 4 00 00 EF
// 28 26 E6 4D 4 00 00 BF
// 28 86 39 4E 4 00 00 5A Boiler Feed (17)
// 28 A0 68 4E 4 00 00 06
// 28 4E D3 4D 4 00 00 8B
// 28 F3 D1 4D 4 00 00 45
// 28 C9 C5 4D 4 00 00 04
//////////////////////////

#define ACK_TIME		500  // number of milliseconds - to wait for an ack
#define RETRY_LIMIT      2

byte payloadSize = BASIC_PAYLOAD_SIZE;
byte NodeID = 27;
word lastCRC;

byte getOTO = false;
byte doPairing = true;
byte doHeatingOn = false;
byte dataChanged = true;
//signed int boilerTrend;
//signed int returnTrend;
signed int tempTrend = 50;	// Should never be zero
int previousCurrentTemp, previousTargetTemp;

static void showString (PGM_P s); // forward declaration

// Interrupt Routines
ISR(TIMER1_COMPA_vect){
	seconds++;
}
ISR(WDT_vect) { Sleepy::watchdogEvent(); }
//

static void showString (PGM_P s) {
    for (;;) {
        char c = pgm_read_byte(s++);
        if (c == 0)
            break;
        if (c == '\n')
            printOneChar('\r');
        printOneChar(c);
    }
}

static byte sendACK() {
payload.boiler_target = settings.maxBoiler;
payload.tracking = settings.tracking;

for (byte t = 1; t <= RETRY_LIMIT; t++) {
    payload.attempts = t;
    rf12_sleep(RF12_WAKEUP);

    if (rf12_recvDone()) {
		showString(PSTR("Discarded: "));	// Flush the buffer
        for (byte i = 0; i < 8; i++) {
            showByte(rf12_buf[i]);
            rf12_buf[i] = 0xFF;			// Paint it over
            printOneChar(' ');
        }
		Serial.println();
//		Serial.flush();
    }

    while (!(rf12_canSend(1))) {	// 0 to ignore background noise, 1 to loop
		showString(PSTR("Airwaves Busy\r"));
		delay(50);
    }

	showString(PSTR("TX Start\n"));
	rf12_sendStart(RF12_HDR_ACK, &payload, payloadSize);
	rf12_sendWait(1);
	showString(PSTR("TX Done\n"));
	Serial.flush();
    byte acked = waitForAck(0); // Wait for the ACK
    if (acked) {
		payload.tick = 0;
        payloadSize = BASIC_PAYLOAD_SIZE;   // Packet was ACK'ed by someone
		if (payload.packetType == 1) payload.packetType = 0;	// Repeated data flag
        for (byte i = 0; i < rf12_len; i++) {
            showByte(rf12_buf[i]);
            printOneChar(' ');
        }
		Serial.println();
//		if (rf12_len > 0) {
//        	payload.ackKey = rf12_buf[3];	// Acknowledge packet using return payload
//			showString(PSTR("Ack Key="));
//			Serial.println(rf12_buf[3]);
//        } else payload.ackKey = 85;

        if (rf12_len > 1) {
        	payload.ackKey = rf12_buf[3];	// Acknowledge packet using return payload
			showString(PSTR("Ack Key="));
			Serial.println(rf12_buf[3]);

			dataChanged = true;
        	unsigned int post = (uint16_t)setbackMax;
			if (rf12_len > 3) {
				post = ( (rf12_buf[6] << 8) | rf12_buf[5] );
				showString(PSTR("Post="));
				Serial.println(post);
			}
			showString(PSTR("Flag="));
			Serial.println(rf12_buf[4]);

            if ((rf12_len + 5) > sizeof payload.messages) rf12_len = (sizeof payload.messages - 5);

			byte q;
            for (q = 0; q < (rf12_len + 5); q++) {
                payload.messages[q] = (byte)rf12_buf[q];	// Return command stream with next packet
                Serial.print(rf12_buf[q], HEX); printOneChar(' ');
            }
			Serial.println();
            payloadSize = BASIC_PAYLOAD_SIZE + (rf12_len + 5);
			byte* p = &settings.start;
			byte i;
            Serial.print(q);
      		showString(PSTR(" Payload length:"));
			Serial.println(payloadSize);

			switch (rf12_buf[4]) {
				case 1:
		            	showString(PSTR("Report eeprom:"));
						for (i = 0; i < sizeof settings; i++) {
							q++;
							payload.messages[q] = p[i];
							payloadSize++;
							Serial.print(p[i]); printOneChar(' ');
						}
                		Serial.print(q);
                  		showString(PSTR(" Payload length:"));
						Serial.println(payloadSize);
						break;

				case 10:
                  		settings.tracking = false;
                		needSetback = true;
                		dataChanged = true;		// Cause a packet send
                  		showString(PSTR("Tracking off\n"));
                  		break;

				case 11:
                  		settings.tracking = true;
                  		doPairing = true;
                		needSetback = false;
						delaySeconds = elapsedSeconds;
                		dataChanged = true;		// Cause a packet send
                  		showString(PSTR("Tracking on\n"));
                  		break;

				case 12:
// WIP
						if ( delaySeconds < (elapsedSeconds + (uint32_t)post) ){
                			needSetback = true;
                			delaySeconds = elapsedSeconds + (uint32_t)post;
                		} 
                		dataChanged = true;		// Cause a packet send
                		Serial.print(post);
                  		showString(PSTR(" seconds of Setback\n"));
                  		break;

				case 13:
                		needSetback = false;
                		delaySeconds = elapsedSeconds + (uint32_t)post;
                		Serial.print(post);
                  		showString(PSTR(" seconds without Setback\n"));
                  		break;

				case 14:
                		doHeatingOn = true;
                  		showString(PSTR("Salus heating set to on\n"));
                  		break;

				case 19:
						settings.SalusAddress = post;
                  		showString(PSTR("Salus address is now "));
                  		Serial.println(settings.SalusAddress);
                  		break;

				case 20:
                  		settings.WatchSALUS = false;
                  		showString(PSTR("WatchSALUS off\n"));
                  		break;

				case 21:
                  		settings.WatchSALUS = true;
                  		showString(PSTR("WatchSALUS on\n"));
                  		break;

				case 22:
						if (rf12_len == 2) {
                        	Serial.print(settings.salusTX);
							settings.salusTX = rf12_buf[4];
                        	showString(PSTR(" Setting Salus TX level:"));
                        	Serial.println(settings.salusTX);
                        }
						break;

/*				case 85:	// Reserved													*/


				case 98:	// Capture OTO code
						getOTO = true;
						showString(PSTR("Learning OTO codes\n"));
						doPairing = true;
						break;

				case 99:
						showString(PSTR("Saving settings to eeprom\n"));
						saveSettings();
						break;

				case 100:
						if ((rf12_len > 3) && (post < tenK)) {
                        	Serial.print(settings.maxBoiler);
							settings.maxBoiler = post;
                        	showString(PSTR(" Setting Boiler Feed Threshold:"));
                        	Serial.println(settings.maxBoiler);
                        }
						break;

/*				case 170:	// Reserved													*/

				case 151:	// Adjust burn time.
						if (rf12_len > 3) {
                  			Serial.print(settings.burnTime1);
							settings.burnTime1 = post;
                  			showString(PSTR(" burnTime1:"));
                  			Serial.println(settings.burnTime1);
                  		}
						break;

				case 152:	// Adjust burn time.
						if (rf12_len > 3) {
                  			Serial.print(settings.burnTime2);
							settings.burnTime2 = post;
                  			showString(PSTR(" burnTime2:"));
                  			Serial.println(settings.burnTime2);
                  		}
						break;

				case 153:	// Adjust burn time.
						if (rf12_len > 3) {
                  			Serial.print(settings.burnTime3);
							settings.burnTime3 = post;
                  			showString(PSTR(" burnTime3:"));
                  			Serial.println(settings.burnTime3);
                  		}
						break;

				case 160:	// Fake Boiler temperature
						if (rf12_len > 3) {
                  			Serial.print(payload.BoilerFeed);
							payload.BoilerFeed = post;
                  			showString(PSTR(" BoilerFeed:"));
                  			Serial.println(payload.BoilerFeed);
                  		}
						break;

				case 200:	// Fake the current temperature
						if ( (rf12_len > 3) && (post < 5000) ) {
                      		Serial.print(payload.currentTemp);
							previousCurrentTemp = payload.currentTemp;
							payload.currentTemp = post;
                      		payload.packetType = 0;	// Faked current Temp
                      		showString(PSTR(" Faked current temperature "));
                      		Serial.println(payload.currentTemp);
                        }
						break;

				case 201:	// Fake the target temperature
						if ( (rf12_len > 3) && (post < 5000) ) {
                      		Serial.print(payload.targetTemp);
							payload.targetTemp = post;
                      		payload.packetType = 0;	// Faked target Temp
                      		showString(PSTR(" Faked target temperature "));
                      		Serial.println(payload.targetTemp);
                        }
						break;
/*
				case 255:
                  		if ( (rf12_len == 2) && (rf12_buf[4] == 255) ) {
                  			showString(PSTR("Waiting for Watchdog\n"));
                  			delay(tenK);		// Wait for watchdog
                  		}
                  		break;
*/
				default:
						payload.ackKey = 170;	// Indicate an issue
						dataChanged = false;
						Serial.print(rf12_buf[4]);
						showString(PSTR(" Alert Unknown Command\n"));
                      	break;

                  } // end switch
        } else payload.ackKey = 85;

		return t;
      }
//	  Sleepy::loseSomeTime(500);
   }
  return 0;
} // sendACK

static byte waitForAck(byte t) {
    MilliTimer ackTimer;
    while (!ackTimer.poll(ACK_TIME)) {
        if (rf12_recvDone()) {
            rf12_sleep(RF12_SLEEP);

             Serial.print((ACK_TIME) - ackTimer.remaining());
             showString(PSTR("ms RX "));

            if (rf12_crc == 0) {                          // Valid packet?
                // see http://talk.jeelabs.net/topic/811#post-4712
				if (rf12_hdr == (RF12_HDR_DST | RF12_HDR_CTL | NodeID)) {
                     showString(PSTR("ACK "));
                    return 1;
                }  else {
                	Serial.print(rf12_hdr, HEX); printOneChar(' ');
                	 Serial.print((RF12_HDR_DST | RF12_HDR_CTL | NodeID), HEX);
                     showString(PSTR(" Unmatched: "));             // Flush the buffer
                    for (byte i = 0; i < 8; i++) {;
                        showByte(rf12_buf[i]);
                        rf12_buf[i] = 0xFF;              // Paint it over
                        printOneChar(' ');
                    }
                    showStats();
                }
            } else {
				showString(PSTR("Bad CRC"));
                payload.badCRC++;
            }
			Serial.println();Serial.flush();
        }
        set_sleep_mode(SLEEP_MODE_IDLE);   // Wait a while for the reply?
        sleep_mode();
    }

	Serial.print(ACK_TIME);
	showString(PSTR("ms ACK Timeout\n"));
	Serial.flush();

    return 0;
} // waitForAck

static void saveSettings () {
    settings.start = 0x55;
    settings.spare = 0;
    settings.crc = calcCrc(&settings, sizeof settings - 2);
    // this uses 170 bytes less flash than eeprom_write_block(), no idea why
    byte* p = &settings.start;
    for (byte i = 0; i < sizeof settings; ++i) {
        if (eeprom_read_byte(SETTINGS_EEPROM_ADDR + i) != p[i]) {
            eeprom_write_byte(SETTINGS_EEPROM_ADDR + i, p[i]);
			wdt_reset();		// Hold off Watchdog: Eeprom writing is slow...
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
    showString(PSTR("Settings CRC "));
    if (crc) {
        showString(PSTR("is bad, defaulting\n"));
        Serial.println(crc, HEX);
        settings.WatchSALUS = true;
        settings.maxBoiler = 5000;
        settings.burnTime3 = 60 * 3;
        settings.burnTime2 = 60 * 15;
        settings.burnTime1 = 60 * 19;
    } else {
         showString(PSTR("is good\n"));
    }
    showString(PSTR("Boiler threshold:"));
    Serial.println(settings.maxBoiler);
    showString(PSTR("Burn1 Time:"));
    Serial.println(settings.burnTime1);
//    burnTime1 = (uint32_t)settings.burnTime1;


} // loadSettings

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
//         Serial.print((word) value, DEC);
}
static void showWord (unsigned int value) {
//    if (config.output & 0x1) {
        showByte (value >> 8);
        showByte (value);
//    } else
//         Serial.print((word) value);
}
/*
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
*/
static word calcCrc (const void* ptr, byte len) {
    word crc = ~0;
    for (byte i = 0; i < len; ++i)
        crc = _crc16_update(crc, ((const byte*) ptr)[i]);
    return crc;
}

void checkSetback () {	// Radio needs to be in Salus mode
        backCount++;

/*#if !DEBUG
    	rf12_sendStart(0, &setPairing, 5);			// Issue a OTO pairing request
    	rf12_sendWait(1);							// Wait for transmission complete.
    	rf12_sendStart(0, &setPairing, 5);			// and again for luck
        rf12_sendWait(1);							// Wait for transmission complete.
#endi*/
        if (!(needSetback)) {
            if (setback) {
		        rf12_sleep(RF12_WAKEUP);					// All set, wake up radio
        		payload.tick = (uint8_t)(seconds - elapsedSeconds);
#if !DEBUG
                rf12_sendStart(0, &setBack0, 5);			// Issue a OTO to cancel setback.
				rf12_sendWait(1);							// Wait for transmission complete.
                rf12_sendStart(0, &setBack0, 5);			// Issue a OTO to cancel setback.
                rf12_sendWait(1);							// Wait for transmission complete.
#endif
                showString(PSTR("Setback Released\n"));
        		rf12_sleep(RF12_SLEEP);
                backCount = 0;								// OTO every 10 mins
                payload.setBack = 0;
                setback = false;
                dataChanged = true;
            }
        } else {
            if (!(setback)) {
        		rf12_sleep(RF12_WAKEUP);
        		payload.tick = (uint8_t)(seconds - elapsedSeconds);
#if !DEBUG
                rf12_sendStart(0, &setBackA, 5);			// Issue a OTO to setback.
                rf12_sendWait(1);							// Wait for transmission complete.
                rf12_sendStart(0, &setBackA, 5);			// Issue a OTO to setback.
                rf12_sendWait(1);							// Wait for transmission complete.
#endif
                showString(PSTR("Setback Issued\n"));
        		rf12_sleep(RF12_SLEEP);
                backCount = 0;								// OTO every 10 mins
                payload.setBack = 1;
                setback = true;
                dataChanged = true;
            }
        }
        // This code refreshes the setback condition regularly - required by Salus kit
        if (backCount >= 60) {	// every 10 mins
        	rf12_sleep(RF12_WAKEUP);
            if (setback) {
#if !DEBUG
                rf12_sendStart(0, &setBackA, 5);			// Issue a OTO to refresh setback.
#endif
                payload.setBack = 1;
            } else {
#if !DEBUG
                rf12_sendStart(0, &setBack0, 5);			// Issue a OTO to refresh null setback.
#endif
                payload.setBack = 0;
            }
#if !DEBUG
            rf12_sendWait(1);                                             // Wait for transmission complete.
#endif
        	rf12_sleep(RF12_SLEEP);
            showString(PSTR("Setback Refreshed\n"));
            backCount = 0;
        }
        Serial.flush();
}

void setup () {
// Setup WatchDog
	wdt_reset();   			// First thing, turn it off
	MCUSR = 0;
	wdt_disable();
	wdt_enable(WDTO_8S);   // enable watchdogtimer

// Set up timer1 interrupt at 1Hz
  	TCCR1A = 0;								// Set TCCR1A to 0
  	TCCR1B = 0;								// Same for TCCR1B
  	TCNT1  = 0;								// Counter value to 0
  	// 1hz increments
  	OCR1A = 15624;							// = (16*10^6) / (1*1024) - 1 (must be <65536)

  	TCCR1B |= (1 << WGM12);					// Activate CTC mode

  	TCCR1B |= (1 << CS12) | (1 << CS10);	// Set 1024 prescaler

  	TIMSK1 |= (1 << OCIE1A);				// Timer1 compare interrupt

   Serial.begin(115200);
   Serial.print((__DATE__));
   showString(PSTR(" "));
   Serial.println((__TIME__));
#if RF69_COMPAT
	payload.ackKey = 0;
   showString(PSTR("RFM69x "));
#else
	payload.ackKey = 0;
	showString(PSTR("\nRFM12x "));
#endif
	Serial.print(SALUSFREQUENCY);
	showString(PSTR(" Heating monitor:"));
	rf12_configDump();
	loadSettings();
	showString(PSTR("Salus address "));
	Serial.println(settings.SalusAddress);
 
 #if RF69_COMPAT
   showString(PSTR("RFM69x "));
#else
   showString(PSTR("RFM12x "));
#endif
	Serial.println();
	Serial.flush();
	
    payload.lowestTemp = (uint16_t)0;	
	payload.currentTemp = (uint16_t)0;
	payload.targetTemp = (uint16_t)0;	// Should turn on heating if tracking enabled
	payload.packetType = 0;
	payload.BoilerFeed = ~0;
	payload.salusAddress = ~0;          // Until we know better
	payload.salusCommand = ~0;          // ditto

//	pinMode(17, OUTPUT);      // Set the pin, AIO4 - Power the DS18B20's
//	digitalWrite(17, HIGH);   // Power up the DS18B20's
//	delay(200);

/// Configure the DS18B20 ///
	ds.reset();               // Set for 12 bit measurements //
	ds.skip();                // Next command to all devices

	ds.write(0x4E);           // Write to Scratch Pad
	ds.write(0x7F);           // Set T(h)
	ds.write(0x80);           // Set T(l)
	ds.write(0x7F);           // Set Config 12 bit


/// Copy config to on-chip EEProm, only required on first use of a new DS18B20
/*
	ds.reset();
	ds.skip();                // Next command to all devices
	ds.write(0x48);           // Set Config
*/
	delay(1000);

	setbackTimer = elapsedSeconds + setbackMax;
/*
settings.SalusAddress = 300;
Serial.println(settings.SalusAddress);
Serial.println((byte)settings.SalusAddress);
Serial.println((byte)settings.SalusAddress >> 8);
*/

  } //  Setup

static void showStats() {
#if RF69_COMPAT
	showString(PSTR(" a="));
	Serial.print(RF69::afc);                        // TODO What units is this count?
	showString(PSTR(" f="));
	Serial.print(RF69::fei);                        // TODO What units is this count?
	showString(PSTR(" l="));
	Serial.print(RF69::lna >> 3);
	showString(PSTR(" t="));
	Serial.print((RF69::readTemperature(-10)));
	showString(PSTR(" ("));
	Serial.print(RF69::rssi >> 1);
	if (RF69::rssi & 0x01)  showString(PSTR(".5"));
		showString(PSTR("dB)"));
		Serial.println();
		Serial.flush();
#endif
	return;
}
// https://deepwiki.com/matmunk/DS18B20/5.3-error-handling-and-debugging
unsigned int getTemp(byte* sensor) {
	byte i;
	byte present = 0;
	byte data[12];
	present = ds.reset();
	if (present) {		
		ds.select(sensor);  
		ds.write(0xBE);						// Request Scratchpad
	} else {
		showString(PSTR("DS18B20 not present: "));
		for ( i = 0; i < 8; i++) {		
			Serial.print( (sensor[i]), HEX);
		}
		return 1;
	}
/*
	showString(PSTR("Addr:"));	
	for ( i = 0; i < 8; i++) {		
		Serial.print( (sensor[i]), HEX);
		showString(PSTR(" "));
	}
*/
/*	
	showString(PSTR("Data: "));
	Serial.print(present, HEX);
	showString(PSTR(" "));
*/
	for ( i = 0; i < 9; i++) {           // we need 9 bytes
		data[i] = ds.read();
//		Serial.print(data[i], HEX);
//		showString(PSTR(" "));
	}

//	showString(PSTR(" CRC="));
//	Serial.println(OneWire::crc8(data, 8), HEX);


  // convert the data to actual temperature

  long int raw = (data[1] << 8) | data[0];
// Assuming 12 bit precision
  raw = raw * 100;
  raw = raw >> 4;
//   showString(PSTR(" Decode: ")); Serial.println(raw);
  return (raw); // return t*100
}

byte payloadReady = false;

static void waitRF12() {
	byte goodAddr = true;
	uint32_t t = elapsedSeconds + uint32_t(1);
    while (t > seconds) {
		wdt_reset();		// Hold off Watchdog
    	if( settings.WatchSALUS ) {
    		if( rf12_recvDone() )	{
    			if( (rf12_buf[0] == 212) /*&& (rf12_buf[1] >= 160) && (rf12_buf[1] != 255)*/ ) {
					Serial.print(elapsedSeconds);
					showString(PSTR("s Salus Packet\n"));
        			Serial.flush();
            		for (byte i = 1; i < 15; i++) {
            			payload.messages[i] = rf12_buf[i];
                		showByte(rf12_buf[i]);
						printOneChar(' ');
           			 }
           			 payloadSize = BASIC_PAYLOAD_SIZE + 12;	// Forward Salus packet in next Jee packet

					showString(PSTR("\nSalus II Device:"));  Serial.flush();
            		Serial.println(rf12_buf[1]);  // Device type
            		unsigned int s =  (rf12_buf[3] << 8) | rf12_buf[2];   // Guessing at a 16 bit address
            		if (s != settings.SalusAddress) {
            			showString(PSTR("Unknown address:"));
            			Serial.print(s);
            			goodAddr = false;            		
            		} else {
            			showString(PSTR("Addr:"));
            		}
            		payload.salusAddress = s;   						// Guessing at a 16 bit address
            		
            		showString(PSTR(" Command:"));
            		payload.salusCommand = rf12_buf[4];
            		Serial.print(payload.salusCommand);
     
           			byte csum = 0; 
           			byte goodCRC = false;
             		switch (rf12_buf[1]) {
             			case 16: // 3 byte payload from thermostat, on/off command
             				if ( (rf12_buf[1] + rf12_buf[2]) == rf12_buf[3] )
								showString(PSTR(" Checksum is good"));
								else {
									showString(PSTR(" Checksum is bad"));
									break;
								}
								showString(PSTR("\nHeating"));
								if ( rf12_buf[2] == 1) showString(PSTR(" on"));
								else showString(PSTR(" off"));
								Serial.println();
							 	break;
                		case 165: // 12 byte payload from thermostat, temperature details
/*
Example packet A5 10 00 20 08 07 F4 01 6C 07 32 7E
Switches on the heating
Where A5 is the thermostat device type.
10 00 is the device id
20 is command?
08 07 is 16 bits, low order first flips to 0x708 equals 18.00 degrees, the current temperature at the thermostat
F4 01 is 16 bits, low order first flips to 0x1F4 equals 5.00 degrees, the lowest temperature in the thermostat program
6C 07 is 16 bits, low order first flips to 0x76C equals 19.00 degrees, the target temperature in the thermostat program
32 Not known
7E is a simple checksum of the preceding 11 bytes
*/

                			if ( !(previousCurrentTemp) ) previousCurrentTemp = ((rf12_buf[10] << 8) | rf12_buf[9]);
							else previousCurrentTemp = payload.currentTemp;

                			if ( !(previousTargetTemp) ) previousTargetTemp = ((rf12_buf[10] << 8) | rf12_buf[9]);
							else previousTargetTemp = payload.targetTemp;

							// First pass default to target temperature
            				showString(PSTR(" Checksum:0x"));
            				Serial.print(rf12_buf[12], HEX);
            				// Calculate checksum
            				for (byte i = 1; i < 12; i++) {
            					csum = csum + rf12_buf[i];
            				}
//             				showString(PSTR("csum:0x"));
//           					Serial.println(csum, HEX);
           					if (csum == rf12_buf[12] ) {
           						showString(PSTR(" Checksum good"));
           						goodCRC = true;
								Serial.println();
           					} else {
           					 	showString(PSTR(" Checksum bad"));
								Serial.println();
           					}
           					if (goodCRC && goodAddr) {
               					payload.currentTemp = ((rf12_buf[6] << 8) | rf12_buf[5]);
                    			showString(PSTR("Current="));  Serial.print(payload.currentTemp);
                    			payload.lowestTemp = ((rf12_buf[8] << 8) | rf12_buf[7]);
                    			showString(PSTR(" Lowest="));   Serial.print(payload.lowestTemp);
                    			payload.targetTemp = ((rf12_buf[10] << 8) | rf12_buf[9]);
                    			showString(PSTR(" Target="));   Serial.println(payload.targetTemp);
                    			payload.packetType = 1;      	// Indicate new data
								payloadReady = true;
								showString(PSTR("payload is ready\n"));
							}
							delay(3000);	// Wait for repeated Salus packets to pass
//							return;
                            break;

                		case 166:   // OTO One Touch Override
            				showString(PSTR(" Checksum:"));
            				Serial.print(rf12_buf[5], HEX);                			
                    		printOneChar(' ');
                    		Serial.println(rf12_buf[5]);
                    		if (getOTO) {
                        		settings.addrOTO = payload.salusAddress;
                        		settings.checkOTO = (rf12_buf[5] - rf12_buf[4]);
                        		showString(PSTR(" Learned OTO Offset "));  Serial.println(settings.checkOTO);  Serial.flush();
                        		getOTO = false; // Learn no more
                    /*		} else if ((payload.salusAddress == settings.addrOTO) && ((rf12_buf[5]) - rf12_buf[4] == settings.checkOTO)) {
                        		showString(PSTR(" One Touch Override Matched"));
                        		showString(PSTR(", Relaying OTO\n"));  Serial.flush();
                        		// Update checksum // byte commandOTO[] = {166, 163, 106, 0, 179};
                        		commandOTO[4] = (commandOTO[4] - commandOTO[3]) + rf12_buf[4];
                        		commandOTO[3] = rf12_buf[4];              // Copy over command byte
                        		//
                        		Serial.print(commandOTO[3]); printOneChar(':');  Serial.print(commandOTO[4]);  Serial.flush();
                        		rf12_sleep(RF12_WAKEUP);
#if !DEBUG
                        		rf12_sendStart(0, &commandOTO, 5);               // Forward the OTO command.
                        		rf12_sendWait(1);                                // Wait for transmission complete.
#endif
                        		rf12_sleep(RF12_SLEEP);
                    */
                    		} 
                    /*
                    		else  {
                    			showString(PSTR(" One Touch Override Unknown\n")); Serial.flush();
                    		}
                    */
                    		break;

                    	case 199:   // JeeStat OTO One Touch Direct Override
                    		if ( rf12_buf[3] == 19 && rf12_buf[4] == 53 ) {	// My magic numbers
                				showString(PSTR(" Jee:"));
                				Serial.print(rf12_buf[3]);
                				printOneChar(',');
                				Serial.print(rf12_buf[4]);
                				printOneChar(',');
                       			Serial.print(rf12_buf[5]);
                				needSetback = true;
                				delaySeconds = elapsedSeconds + (uint32_t)rf12_buf[5];
                				Serial.print( rf12_buf[5] );
                  				showString(PSTR(" seconds of Setback\n"));

                       		}
                        	break;

                		default:
                    		showString(PSTR("\nUnknown "));
                    		for (byte i = 0; i < 8; i++) {;
                        		showByte(rf12_buf[i]);
                        		printOneChar(' ');
                    		}
                    		Serial.flush();
                    		break;
                		}
                		showStats();
                		Serial.flush();
	        		} // rf12_buf
	    		} // rf12_recvDone
    		} // settings.WatchSALUS
    	} // while
    } // waitRF12

byte salusMode = false;

void loop () {
	elapsedSeconds = seconds;	// Use same timestamp throughout loop
/*
	showString(PSTR("Elapsed "));
	Serial.print(millis());
	showString(PSTR(" Next TX "));
	Serial.print(nextScheduled);
	showString(PSTR("-"));
	Serial.println(elapsedSeconds);
*/
/*
 * Setup to receive Salus transmissions
 */
	if (!(salusMode)) {
    	for (byte i = 0; i < 66; i++) rf12_buf[i] = 0;              // Clear buffer
    	rf12_initialize (SALUSID, RF12_868MHZ, 212, SALUSFREQUENCY);// 868.3khz
    	rf12_sleep(RF12_SLEEP);                                     // Sleep while we tweak things
    	rf12_skip_hdr(2);                                           // Omit Jeelib header 2 bytes on transmission
    	rf12_leader(0xAA);											// Apply typical framing
    	rf12_fix_len(15);                                           // Maximum fixed length packet size.

#if RF69_COMPAT
    	RF69::control(REG_BITRATEMSB | 0x80, 0x34);                 // 2.4kbps
    	RF69::control(REG_BITRATELSB | 0x80, 0x15);
    	RF69::control(REG_BITFDEVMSB | 0x80, 0x03);                 // 60kHz freq shift
    	RF69::control(REG_BITFDEVLSB | 0x80, 0xD7);
    	rfapi.txPower = settings.salusTX;							// Set Salus TX power level
#else
		rf12_control(0xC040);                                       // set low-battery level to 2.2V
    	rf12_control(RF12_DATA_RATE_2);                             // 0xC691 app 2.4kbps
    	rf12_control(0x9830 | (settings.salusTX & 0x07) );			// 60khz freq shift, TX
#endif
        salusMode = true;

    	if (doPairing) {
    		rf12_sendStart(0, &setPairing, 5);			// Issue a OTO pairing request
        	rf12_sendWait(1);							// Wait for transmission complete.
        	rf12_sendStart(0, &setPairing, 5);			// and again for luck
        	rf12_sendWait(1);							// Wait for transmission complete.
        	doPairing = false;
    	}
    	
     	if (doHeatingOn) {
    		rf12_sendStart(0, &setHeatingOn, 12);		// Issue a Controller heating on.
        	rf12_sendWait(1);							// Wait for transmission complete.
        	rf12_sendStart(0, &setHeatingOn, 12);		// and again for luck
        	rf12_sendWait(1);							// Wait for transmission complete.
        	doHeatingOn = false;
    	}
   	
    	
    	
    	
    } //salusMode

	ds.reset();
    ds.skip();              // Next command to all devices
	ds.write(0x44);			// Start all temperature conversions.

	wdt_reset();			// Hold off Watchdog
	
	rf12_sleep(RF12_WAKEUP);
	waitRF12();	// Loop whilst checking radio
//	showString(PSTR("Sleep radio\n")); Serial.flush();
	rf12_sleep(RF12_SLEEP);

	payload.ColdFeed = 0;
	unsigned int temp = getTemp(ColdFeed);
	if (temp < 65520ul) payload.ColdFeed = temp;
/*
	 showString(PSTR("Cold Feed:"));
	 Serial.print(payload.ColdFeed);
	 printOneChar(' ');
*/
	payload.BoilerFeed = 0;
	temp = getTemp(BoilerFeed);
	if (temp < 65520ul) payload.BoilerFeed = temp;
//	boilerTrend = 0;
//	if (previousBoilerFeed) boilerTrend = payload.BoilerFeed - previousBoilerFeed;
//		previousBoilerFeed = payload.BoilerFeed;
/*
         showString(PSTR("Boiler Feed:"));
         Serial.print(payload.BoilerFeed);
		 printOneChar(' ');
*/
//         showString(PSTR(" Boiler trend:"));
//         Serial.println(boilerTrend);

    payload.CentralHeatingReturn= 0;
    temp = getTemp(CentralHeatingReturn);
	if (temp < 65520ul) payload.CentralHeatingReturn = temp;
//        returnTrend = 0;
//        if (previousReturn) returnTrend = payload.CentralHeatingReturn - previousReturn;
//        previousReturn = payload.CentralHeatingReturn;
/*
         showString(PSTR("Heating Return:"));
         Serial.print(payload.CentralHeatingReturn);
	 	 printOneChar(' ');
*/
 //        showString(PSTR(" Return trend:"));
//         Serial.println(returnTrend);

	payload.HWTankTemp = 0;
	temp = getTemp(HWTankTemp);
	if (temp < 65520ul) payload.HWTankTemp = temp;
/*
 		showString(PSTR("HW Tank Temp:"));
		Serial.println(payload.HWTankTemp);
*/
		if (settings.tracking) {

//			Serial.print(payload.currentTemp); showString(PSTR(" currentTemp\n"));
//			Serial.print(payload.targetTemp); showString(PSTR(" targetTemp\n"));

			if (payload.currentTemp >= payload.targetTemp) {	// Backstop
				needSetback = true;
				waitSeconds = elapsedSeconds;
//				Serial.print(payload.currentTemp); showString(PSTR(" Temperature Fine\n"));

			} else
			if (delaySeconds < elapsedSeconds) {
				bool tempChanged;
				int c = payload.currentTemp - previousCurrentTemp;
				if (c) {
					tempTrend = c;
					// Positive if temperature is increasing
//					showString(PSTR("Temp Trend ")); Serial.println(tempTrend);
					previousCurrentTemp = payload.currentTemp;
					tempChanged = true;
				} else {
//					showString(PSTR("Temp trend unchanged ")); Serial.println(tempTrend);
					tempChanged = false;
				}

				if (payload.targetTemp > previousTargetTemp) {
					tempChanged = true;
//			 		showString(PSTR("Target temperature changed\n"));
			 		tempTrend = 50;	// We choose burnTime2
					previousTargetTemp = payload.targetTemp;
				} else
				if (payload.targetTemp < previousTargetTemp) {
					tempChanged = false;
//			 		showString(PSTR("Target temperature changed\n"));
					previousTargetTemp = payload.targetTemp;
				}

				if (tempChanged){
//			 		showString(PSTR("A temperature changed\n"));
					if (tempTrend > 0) {
						waitSeconds = elapsedSeconds + (uint32_t)settings.burnTime2;
//			 			showString(PSTR("burnTime2:")); Serial.println(settings.burnTime2);
					} else {
						waitSeconds = elapsedSeconds + (uint32_t)settings.burnTime1;
//			 			showString(PSTR("burnTime1:")); Serial.println(settings.burnTime1);
					}
				}

//				showString(PSTR("elapsedSeconds ")); Serial.println( (signed long int)(elapsedSeconds) );
//				showString(PSTR("waitSeconds ")); Serial.println( (signed long int)(waitSeconds) );

				bool waiting;
				if (waitSeconds >= elapsedSeconds) {
					waiting = true;
//					Serial.print( (signed long int)(waitSeconds - elapsedSeconds) ); showString(PSTR(" Waiting\n"));
				} else {
					waiting = false;
					waitSeconds = elapsedSeconds;
//					Serial.print( (signed long int)(waitSeconds - elapsedSeconds) );
//			 		showString(PSTR(" Not Waiting\n"));
				}

				if (waiting) needSetback = false;
				else if ( (payload.currentTemp + 50) >= payload.targetTemp) {
					if (payload.BoilerFeed >= settings.maxBoiler) {
//						showString(PSTR("Boiler above threshold:")); Serial.println(payload.BoilerFeed);
						needSetback = true;
					} else {
//				 		showString(PSTR("Boiler below threshold:")); Serial.println(payload.BoilerFeed);
						needSetback = false;
						waitSeconds = elapsedSeconds + (uint32_t)settings.burnTime3;
//                  		showString(PSTR(" burnTime3:")); Serial.println(settings.burnTime3);
					}
				} else {
					needSetback = false;
//					Serial.print(payload.currentTemp); showString(PSTR(" Temperature under target\n"));
				}
			} // if (delaySeconds < elapsedSeconds)

		} else {	// settings.tracking

			needSetback = true;
//			showString(PSTR("Tracking disabled\n"));

		}			// settings.tracking

		checkSetback();

		payload.status = (uint16_t)waitSeconds;
		payload.elapsed = (uint16_t)elapsedSeconds;

        if (payloadReady) {
            dataChanged = true;
 			payloadReady = false;

            if (payload.lowestTemp != payload.targetTemp) { // Typical daytime setting
            	if (payload.currentTemp < payload.targetTemp) payload.underRun++;
                if (payload.currentTemp > payload.targetTemp) payload.overRun++;
                if (payload.currentTemp == payload.targetTemp) payload.onTarget++;
            } else {
                // Clear temperature mismatch counters
                payload.underRun = payload.overRun = payload.onTarget = 0;
            }
        }

        if ((elapsedSeconds >= nextScheduled) || (dataChanged)) {	// approx 60 seconds
        	dataChanged = false;
            payload.count++;
            if (NodeID = rf12_configSilent()) {
   				 rf12_control(0xC040);			// set low battery level to 2.2V
            	salusMode = false;

                showString(PSTR("Node "));
                Serial.print(NodeID);
                showString(PSTR(" sending packet #"));
                Serial.print(payload.count);
                showString(PSTR(" length "));
                Serial.print(payloadSize);
                showString(PSTR(" sizeof "));
                Serial.println(sizeof (struct payload));

                byte tries = sendACK();

        		nextScheduled = elapsedSeconds + minute;

                if (tries) {
                     Serial.print(tries);
                     showString(PSTR(" attempt(s)\n"));
                } else {
                     showString(PSTR("Packet #"));
                     Serial.print(payload.count);
                     showString(PSTR(" Aborted\n"));
                }
                Serial.flush();
        } else {
            while( true ) {
                rf12_sleep(RF12_SLEEP);
                showString(PSTR("RF12 eeprom not valid, run RFxConsole\n"));  Serial.flush();
				wdt_reset();		// Hold off Watchdog
				delay(5000);
            }
        }
    }
/*
//	showString(PSTR("Voltage:"));
    payload.voltage = readVcc();
//    Serial.println(payload.voltage);
	if (payload.voltage > 28) {

    	 showString(PSTR("Looping "));
         Serial.println(++loopCount);
    	 Serial.flush();

    } else {
        rf12_sleep(RF12_SLEEP);
        showString(PSTR("Replace batteries\n"));
    	Serial.flush();
    	cli();
    }
*/
#if DEBUG
    showString(PSTR("DEBUG Enabled - OTO NOT TRANSMITTED")); Serial.println();
#endif
/*
    showString(PSTR("Temp Trend=")); Serial.print(tempTrend);
    showString(PSTR(" elapsedSeconds=")); Serial.print(elapsedSeconds);
    showString(PSTR(" waitSeconds=")); Serial.print(waitSeconds);
    showString(PSTR(" Setback=")); Serial.print(setback);
    showString(PSTR(" needSetback=")); Serial.print(needSetback);
    showString(PSTR(" previousCurrentTemp=")); Serial.print(previousCurrentTemp);
    showString(PSTR(" currentTemp=")); Serial.print(payload.currentTemp);
    showString(PSTR(" targetTemp=")); Serial.print(payload.targetTemp);
    showString(PSTR(" backCount=")); Serial.print(backCount);
    Serial.println();
*/    
//    delay(1000);

} // Loop
