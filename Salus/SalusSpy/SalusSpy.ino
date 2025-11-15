///////////////////////////////////////////////////////////////////////////////
#define RF69_COMPAT      1   // define this to use the RF69 driver i.s.o. RF12
//                           // The above flag must be set similarly in RF12.cpp
//                           // and RF69_avr.h
///////////////////////////////////////////////////////////////////////////////
//
#include <JeeLib.h>
#include "RFAPI.h"		// Define
rfAPI rfapi;			// Declare
#include <avr/sleep.h>
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
volatile unsigned long seconds, elapsedSeconds, nextScheduled, setbackTimer;
byte addr[8];
byte salusOff[] = {SALUSID, OFF, SALUSID | OFF, 90};
/*
    Commands: 00/04 is for 2 degrees setback switch			0b00000000/0b00000100
              08/12 is for 4 degrees setback switch			0b00001000/0b00001100
              16/20 is for 6 degrees setback switch			0b00010000/0b00010100
              24/28 is for Auto degrees setback switch		0b00011000/0b00011100

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

#define ACK_TIME		500  // number of milliseconds - to wait for an ack
#define RETRY_LIMIT      2

byte payloadSize = BASIC_PAYLOAD_SIZE;
byte NodeID = 27;
word lastCRC;

byte getOTO = false;
byte doPairing = true;
byte doHeatingOn = false;
byte dataChanged = true;

static void showString (PGM_P s); // forward declaration

// Interrupt Routines
ISR(TIMER1_COMPA_vect){
	seconds++;
}
ISR(WDT_vect) { Sleepy::watchdogEvent(); }

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
    } else {
         showString(PSTR("is good\n"));
    }

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


		return t;
      	}
      }
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

static word calcCrc (const void* ptr, byte len) {
    word crc = ~0;
    for (byte i = 0; i < len; ++i)
        crc = _crc16_update(crc, ((const byte*) ptr)[i]);
    return crc;
}

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
            		for (byte i = 1; i < 13; i++) {
            			payload.messages[i] = rf12_buf[i];
                		showByte(rf12_buf[i]);
						printOneChar(' ');
           			 }
           			 payloadSize = BASIC_PAYLOAD_SIZE + 13;	// Forward Salus packet in next Jee packet

					showString(PSTR("\nSalus II Device:"));  Serial.flush();
            		Serial.println(rf12_buf[1]);  // Device type
            		unsigned int s =  (rf12_buf[3] << 8) | rf12_buf[2];   // Guessing at a 16 bit address
//            		if (s != settings.SalusAddress) {
//            			showString(PSTR("Unknown address:"));
//            			Serial.print(s);
//            			goodAddr = false;            		
//            		} else {
//            			showString(PSTR("Addr:"));
//            		}
            		showString(PSTR("Addr:"));
            		Serial.print(s);
            		showString(PSTR(" Command:"));
            		payload.salusCommand = rf12_buf[4];
            		Serial.print(payload.salusCommand);
     
           			byte csum = 0; 
           			byte goodCRC = false;
             		switch (rf12_buf[1]) {
             			case 16:
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
                		case 165: // Thermostat
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
							// First pass default to target temperature
            				showString(PSTR(" RT500RF Checksum:0x"));
            				Serial.print(rf12_buf[12], HEX);
            				// Calculate checksum
            				for (byte i = 1; i < 12; i++) {
            					csum = csum + rf12_buf[i];
            				}
//             				showString(PSTR("csum:0x"));
//           					Serial.println(csum, HEX);
           					if (csum == rf12_buf[12] ) {
           						showString(PSTR(" Checksum is good"));
           						goodCRC = true;
								Serial.println();
           					} else {
           					 	showString(PSTR(" Checksum is bad"));
								Serial.println();
           					}
               				payload.currentTemp = ((rf12_buf[6] << 8) | rf12_buf[5]);
                    		showString(PSTR("Current="));  Serial.print(payload.currentTemp);
                    		payload.lowestTemp = ((rf12_buf[8] << 8) | rf12_buf[7]);
                			showString(PSTR(" Lowest="));   Serial.print(payload.lowestTemp);
                			payload.targetTemp = ((rf12_buf[10] << 8) | rf12_buf[9]);
                			showString(PSTR(" Target="));   Serial.println(payload.targetTemp);
                            break;

                		case 166:   // OTO One Touch Override
            				showString(PSTR(" OTO Unit Checksum:"));
            				Serial.print(rf12_buf[5], HEX); 
            				
            				
            				// Calculate checksum     				
            				for (byte i = 1; i < 5; i++) {
            					csum = csum + rf12_buf[i];
            				}
//             				showString(PSTR("csum:0x"));
//           					Serial.println(csum, HEX);
           					if (csum == rf12_buf[5] ) {
           						showString(PSTR(" is good"));
           					} else {
           					 	showString(PSTR(" bad 0x"));
                    			Serial.print(rf12_buf[5], HEX);
           					} 
           					showString(PSTR("\nSetback "));
           					decodeSetback();
           					
                    		if (getOTO) {
                        		settings.addrOTO = payload.salusAddress;
                        		settings.checkOTO = (rf12_buf[5] - rf12_buf[4]);
                        		showString(PSTR(" Learned OTO Offset "));  Serial.println(settings.checkOTO);  Serial.flush();
                        		getOTO = false; // Learn no more
                    		}
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
                				Serial.print( rf12_buf[5] );
                  				showString(PSTR(" seconds of Setback\n"));

                       		}
                        	break;

                		default:
                    		showString(PSTR("\nUnknown Device\n"));
 //                   		for (byte i = 0; i < 8; i++) {;
 //                     		showByte(rf12_buf[i]);
 //                       		printOneChar(' ');
 //                   		}
                    		Serial.println();
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

static void decodeSetback() {
           					
    switch ((rf12_buf[4] >> 3)) {
        case 0:
        	showString(PSTR("2° "));
        	break;
        	
        case 1:
           	showString(PSTR("4° "));
           	break;
           	
        case 2:           							
           	showString(PSTR("6° "));
           	break;
           	
        case 3:
           	showString(PSTR("Auto° "));
           	break; 
        default:
            showString(PSTR("\ndefault "));
            Serial.println(rf12_buf[4] >> 3);
           	break;			           					
    } 
    if ( (rf12_buf[4] & 0b00000100) ) {
    	showString(PSTR("on, Heating off"));
    	} else showString(PSTR("off, Heating on"));
    	
    if ( (rf12_buf[4] & 0b00000010) ) showString(PSTR("set pairing"));
		Serial.println();
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

	Serial.begin(115200); printOneChar('\n');
	Serial.print((__DATE__));
	showString(PSTR(" "));
	Serial.println((__TIME__));
	Serial.print(SALUSFREQUENCY);
	showString(PSTR(" Salus monitor:"));
	rf12_configDump();
	loadSettings();
 
 #if RF69_COMPAT
   showString(PSTR("RFM69x "));
#else
   showString(PSTR("RFM12x "));
#endif
	Serial.println();
	Serial.flush();

/*
 * Setup to receive Salus transmissions
 */
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
    rfapi.RssiToSyncLimit = SALUSPACKET16;
 #else
	rf12_control(0xC040);                                       // set low-battery level to 2.2V
	rf12_control(RF12_DATA_RATE_2);                             // 0xC691 app 2.4kbps
    rf12_control(0x9830 | (settings.salusTX & 0x07) );			// 60khz freq shift, TX
#endif
	Serial.println("Setup complete");
  } //  Setup

void loop () {

	wdt_reset();			// Hold off Watchdog
	
	elapsedSeconds = seconds;	// Use same timestamp throughout loop

	waitRF12();	// Listen to radio
	


} // Loop

