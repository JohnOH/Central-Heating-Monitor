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
 * withholds the heating demand the demand will still be requested by the thermostat if 
 * conditions dictate.
 * 
 * <review> If the Salus thermostat fails to deliver its demand status on its regular
 * schedule then this sketch will cancel the central heating demand.
 *
 * 13/04/2017 Tidy up lots of problems hearing Salus commands
 */
unsigned long minute = 60;	// note the approx 2 seconds delay in code
unsigned long burnTime = 60 * 20;
unsigned long setbackMax = 595;	// Close to 10 minutes 
volatile unsigned long elapsedSeconds, nextScheduled, setbackTimer, delaySeconds;
volatile byte seconds;
ISR(TIMER1_COMPA_vect){
	elapsedSeconds++;
	seconds++;
}

ISR(WDT_vect) { Sleepy::watchdogEvent(); }

unsigned int loopCount;
byte ColdFeed[8] = {0x28,0x53,0x4F,0x4E,0x04,0x00,0x00,0x84};
byte BoilerFeed[8] = {0x28,0x86,0x39,0x4E,0x04,0x00,0x00,0x5A};
byte CentralHeatingReturn[8] = {0x28,0x7F,0xCA,0x4D,0x4,0x0,0x0,0xDE};
byte HWTankTemp[8] = {0x28,0x7F,0xC6,0x4D,0x04,0x00,0x00,0xFF};

byte addr[8];
byte salusOff[] = {SALUSID, OFF, SALUSID | OFF, 90};
byte commandOTO[] = {166, 163, 106, 0, 179};
//byte setBack0[] = {166, 163, 106, 0, 179};
//byte setBack2[] = {166, 163, 106, 4, 183}; // Setback temperature by 2 degrees
byte setBack0[] = {166, 163, 106, 24, 203};
byte setBack1[] = {166, 163, 106, 28, 207}; // Setback temperature by AUTO degrees
byte setback = false; byte needSetback = false;
unsigned int backCount = 65500;
int previousBoilerFeed;
int previousReturn;
//
/////////////////////////////////////////////////////////////////////////////////////
struct payload{                                                                    //
byte command;         // Last command received in ACK
byte badCRC:  4;      // Running count of CRC mismatches
byte tracking:	1;	  // True if we are tracking and capping boiler output
byte setBack: 1;      // True if a setback is pending
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
unsigned int HWTankTemp;
unsigned int boiler_target;
unsigned int overRun;   // Temperature overrun
unsigned int underRun;  // Temperature underrun
unsigned int onTarget;  // Temperature correct
#define BASIC_PAYLOAD_SIZE 30
byte messages[64 - BASIC_PAYLOAD_SIZE];
} payload;
//
/////////////////////////////////////////////////////////////////////////////////////
typedef struct {
    byte start;	// 0x55
    byte WatchSALUS:1;
    byte tracking:1;
    byte spare:6;
    unsigned int maxBoiler;
    unsigned int burnTime;
    byte delayLoop;
    byte salusTX;	// Transmit power when sending OTO commands
    unsigned int addrOTO;
    byte checkOTO;
    word crc;
} eeprom;
static eeprom settings;
/////////////////////////////////////////////////////////////////////////////////////

// Ten off wired DS18B20
// Blk=GND, Blue=DQ Red=Vdd
// 28 7F C6 4D 4 00 00 FF Hot Water Tank
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
#define RETRY_LIMIT      15

byte payloadSize = BASIC_PAYLOAD_SIZE;
byte NodeID = 16;
word lastCRC;

byte getOTO = false;
byte dataChanged = true;
//signed int boilerTrend;
//signed int returnTrend;

static byte sendACK() {
payload.boiler_target = settings.maxBoiler;
payload.tracking = settings.tracking;
  
for (byte t = 1; t <= RETRY_LIMIT; t++) {  
    delay(t * t);                   // Increasing the gap between retransmissions
    payload.attempts = t;
    rf12_sleep(RF12_WAKEUP);
    if (rf12_recvDone()) {
         Serial.print("Discarded: ");             // Flush the buffer
        for (byte i = 0; i < 8; i++) {
            showByte(rf12_buf[i]);
            rf12_buf[i] = 0xFF;                     // Paint it over
            printOneChar(' ');
        }
         Serial.println();
         Serial.flush(); 
    }
      
    while (!(rf12_canSend())) {
         Serial.print("Airwaves Busy");
		delay(50);
    }
        
	Serial.println("TX Start");
	rf12_sendStart(RF12_HDR_ACK, &payload, payloadSize);
	Serial.println("TX Done");
    Serial.flush();      
    byte acked = waitForAck(t * t); // Wait for increasingly longer time for the ACK
    if (acked) {
        payloadSize = BASIC_PAYLOAD_SIZE;   // Packet was ACK'ed by someone
		if (payload.packetType == 2) payload.packetType = 3;	// Repeated data flag
        for (byte i = 0; i < 6; i++) {
            showByte(rf12_buf[i]);
            printOneChar(' ');
        }
		Serial.println();
        if (rf12_buf[2] > 0) {                          // Non-zero length ACK packet?
            payload.command = rf12_buf[3];
        	dataChanged = true;
             Serial.print("Command=");
             Serial.println(rf12_buf[3]);

            if ((rf12_len + 5) > sizeof payload.messages) rf12_len = (sizeof payload.messages - 5);
             
            for (byte i = 0; i < (rf12_len + 5); i++) {
                payload.messages[i] = (byte)rf12_buf[i];     // Return command stream with next packet
                Serial.print(rf12_buf[i], HEX); Serial.print(" ");
                Serial.print((byte)payload.messages[i], HEX); Serial.print(" ");
            }
            Serial.println();
            payloadSize = BASIC_PAYLOAD_SIZE + (rf12_len + 5);
			byte* p = &settings.start;
			byte i;
			switch (rf12_buf[3]) {
				case 0:
		            Serial.println(payloadSize);  
					for (i = 0; i < sizeof settings; i++) {
						payload.messages[((rf12_len + 5) + i)] = p[i];
						payloadSize++;
						Serial.print(p[i]); Serial.print(" ");
					}
					Serial.println();
		            Serial.println(payloadSize);  
					break;
				case 1:
                  		burnTime = burnTime - minute;
                  		Serial.print("BurnTime-:");
                  		Serial.println(burnTime);
                  		break;
				case 2:
                  		burnTime = burnTime + minute;
                  		Serial.print("BurnTime+:");
                  		Serial.println(burnTime);
                  		break;
				case 10:
                  		settings.tracking = false;
                		needSetback = true;
                  		Serial.println("Tracking off");
                  		break;
				case 11:
                  		settings.tracking = true;
                		needSetback = false;
						delaySeconds = elapsedSeconds;
                  		Serial.println("Tracking on");
                  		break;
				case 12:
                		needSetback = true;
                		delaySeconds = elapsedSeconds + setbackMax;
                  		Serial.println("10 minutes Setback");
                  		break;
				case 13:
                		needSetback = false;
                		delaySeconds = elapsedSeconds + setbackMax;
                  		Serial.println("10 minutes without Setback");
                  		break;
/*				case 30:
                  		settings.txSkip = 30;
                  		Serial.println("TX Skip 30");
                  		break;
*/				case 98:  // Capture OTO code
						getOTO = true;
						Serial.println("Learning OTO codes");
						break;
				case 99:
						Serial.println("Saving settings to eeprom");
						saveSettings();
						break;      
				default:
                      if (rf12_buf[3] > 0 && rf12_buf[3] < 10) {
                        settings.delayLoop = rf12_buf[3];
						Serial.print("Delay Loop:");
                        Serial.println(settings.delayLoop);
                        break;
                      }
                      if (rf12_buf[3] > 63 && rf12_buf[3] < 96) {
                        settings.salusTX = rf12_buf[3];
                        Serial.print("Setting Salus OTO TX Power:");
                        Serial.println(settings.salusTX);
                        break;   
                      }
                      if (rf12_buf[3] >= 100 && rf12_buf[3] <= 199) {
                          settings.maxBoiler = (4000 + (rf12_buf[3] - 100) * 20);
                           Serial.print("Setting Boiler Feed Threshold:");
                           Serial.println(settings.maxBoiler);
                          break;  
                      }
                      if (rf12_buf[3] >= 200 && rf12_buf[3] <= 255) {
                      		payload.packetType = 1;	// Faked current Temp
                      		payload.currentTemp = 1800 + ((rf12_buf[3] - 200) * 10);
                      		Serial.print("Faked current temperature ");
                      		Serial.println(payload.currentTemp);
							break;
                      }
						Serial.println("Unknown Command");
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

             Serial.print((ACK_TIME + t) - ackTimer.remaining());
             Serial.print("ms RX ");
            
            if (rf12_crc == 0) {                          // Valid packet?
                // see http://talk.jeelabs.net/topic/811#post-4712
                if (rf12_hdr == (RF12_HDR_DST | RF12_HDR_CTL | NodeID)) {
                     Serial.print("ACK ");
                    return 1;            
                } else {
                     Serial.print("Unmatched: ");             // Flush the buffer
                    for (byte i = 0; i < 8; i++) {;
                        showByte(rf12_buf[i]);
                        rf12_buf[i] = 0xFF;              // Paint it over
                        printOneChar(' ');
                    }
                    showStats();                                
                }
            } else {
                 Serial.print("Bad CRC");
                payload.badCRC++;
            }
             Serial.println();  Serial.flush();           
        } 
        set_sleep_mode(SLEEP_MODE_IDLE);   // Wait a while for the reply?
        sleep_mode();
    }
//    printOneChar(' ');
     Serial.print(ACK_TIME + t);
     Serial.println("ms ACK Timeout");
     Serial.flush();
    return 0;
} // waitForAck

static void saveSettings () {
    settings.start = 0x55;
    settings.spare = 0;
	settings.burnTime = (uint16_t)burnTime;
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
    Serial.print("Settings CRC ");
    if (crc) {
        Serial.println("is bad, defaulting");
        Serial.println(crc, HEX);
        settings.WatchSALUS = true;
        settings.maxBoiler = 4500;
        settings.burnTime = (uint16_t)burnTime;
    	settings.delayLoop = 58;
    } else {
         Serial.println("is good");
    }
    Serial.print("Boiler threshold:");
    Serial.println(settings.maxBoiler);
    Serial.print("Burn Time:");
    Serial.println(settings.burnTime);
    burnTime = uint32_t(settings.burnTime);
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

void checkSetback () {
        backCount++;
        if (!(needSetback)) {
            if (setback) {
		        rf12_sleep(RF12_WAKEUP);                                  // All set, wake up radio
                rf12_sendStart(0, &setBack0, 5);                          // Issue a OTO to cancel setback.
                rf12_sendWait(1);                                         // Wait for transmission complete.
                Serial.println("Setback Cancelled");
                rf12_sendStart(0, &setBack0, 5);                          // Issue a OTO to cancel setback.
                rf12_sendWait(1);                                         // Wait for transmission complete.
        		rf12_sleep(RF12_SLEEP);
				delaySeconds = delaySeconds + minute; 					// Debounce
                backCount = 0;                                            // OTO every 20 mins
                payload.setBack = 0;
                setback = false;
                dataChanged = true;
            }
        } else {
            if (!(setback)) {
        		rf12_sleep(RF12_WAKEUP);
                rf12_sendStart(0, &setBack1, 5);                          // Issue a OTO to setback.
                rf12_sendWait(1);                                         // Wait for transmission complete.
                Serial.println("Setback Issued");
                rf12_sendStart(0, &setBack1, 5);                          // Issue a OTO to setback.
                rf12_sendWait(1);                                         // Wait for transmission complete.
        		rf12_sleep(RF12_SLEEP);
				delaySeconds = delaySeconds + minute; 					// Debounce
                backCount = 0;                                            // OTO every 20 mins
                payload.setBack = 1;
                setback = true;
                dataChanged = true;
                setbackTimer = elapsedSeconds + setbackMax;
            } 
            else if ((settings.tracking) && (elapsedSeconds >= setbackTimer)) {
		        rf12_sleep(RF12_WAKEUP);                                  // All set, wake up radio
                rf12_sendStart(0, &setBack0, 5);                          // Issue a OTO to cancel setback.
                rf12_sendWait(1);                                         // Wait for transmission complete.
                Serial.println("Setback Cancelled");
                rf12_sendStart(0, &setBack0, 5);                          // Issue a OTO to cancel setback.
                rf12_sendWait(1);                                         // Wait for transmission complete.
        		rf12_sleep(RF12_SLEEP);
                backCount = 0;                                            // OTO every 20 mins
				delaySeconds = delaySeconds + minute; 					// Debounce
                payload.setBack = 0;
                setback = false;
                needSetback = false;
                dataChanged = true;
            }
        }
        if (backCount > setbackMax) {
        	rf12_sleep(RF12_WAKEUP);
            if (setback) {
                rf12_sendStart(0, &setBack1, 5);                          // Issue a OTO to refresh setback.
                payload.setBack = 1;
            } else {
                rf12_sendStart(0, &setBack0, 5);                          // Issue a OTO to refresh null setback.
                payload.setBack = 0;
            }
            rf12_sendWait(1);                                             // Wait for transmission complete.
        	rf12_sleep(RF12_SLEEP);
            Serial.println("\nSetback Refreshed\n");
            backCount = 0;                                                // OTO every 20 mins
        }
        Serial.flush();
}

void setup () {
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
   Serial.print(" ");
   Serial.println((__TIME__));
#if RF69_COMPAT
	payload.command = 9;	// Indicate RFM69
   Serial.print("RFM69x ");
#else
	payload.command = 2;	// Indicate RFM12B
	Serial.print("\nRFM12x ");
#endif
	Serial.print(SALUSFREQUENCY);  
	Serial.print(" Heating monitor:");
	rf12_configDump();
	loadSettings();
 #if RF69_COMPAT
   Serial.print("RFM69x ");
#else
   Serial.print("RFM12x ");
#endif
	Serial.flush();
	payload.packetType = 0;
	payload.BoilerFeed = ~0;
	payload.salusAddress = ~0;          // Until we know better
	payload.salusCommand = ~0;          // ditto

	pinMode(17, OUTPUT);      // Set the pin, AIO4 - Power the DS18B20's
	digitalWrite(17, HIGH);   // Power up the DS18B20's
	delay(200);

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
	delay(200);
	setbackTimer = elapsedSeconds + setbackMax;

// Setup WatchDog
	wdt_reset();   			// First thing, turn it off
	MCUSR = 0;
	wdt_disable();
	wdt_enable(WDTO_8S);   // enable watchdogtimer

  } //  Setup
  
static void showStats() {
#if RF69_COMPAT
             Serial.print(" a=");
             Serial.print(RF69::afc);                        // TODO What units is this count?
             Serial.print(" f=");
             Serial.print(RF69::fei);                        // TODO What units is this count?
             Serial.print(" l=");
             Serial.print(RF69::lna >> 3);
             Serial.print(" t=");
             Serial.print((RF69::readTemperature(-10)));        
             Serial.print(" (");
             Serial.print(RF69::rssi >> 1);
            if (RF69::rssi & 0x01)  Serial.print(".5");
             Serial.print("dB)");
#endif
             Serial.println();
             Serial.flush();
            return;
}

unsigned int getTemp(byte* sensor) {
	byte i;
	byte present = 0;
	byte data[12]; 

	ds.reset();
	ds.select(sensor);    
	ds.write(0xBE);                                            // Request Scratchpad

	Serial.print("Data = ");
	Serial.print(present,HEX);
	Serial.print(" ");

	for ( i = 0; i < 9; i++) {           // we need 9 bytes
		data[i] = ds.read();
		Serial.print(data[i], HEX);
		Serial.print(" ");
	}
/*
   Serial.print(" CRC=");
   Serial.print(OneWire::crc8(data, 8), HEX);
   Serial.println();
*/
  // convert the data to actual temperature

  long int raw = (data[1] << 8) | data[0];
// Assuming 12 bit precision
  raw = raw * 100;
  raw = raw >> 4;
  return (raw); // return t*100
}

byte payloadReady = false;

static void waitRF12() {
	seconds = 0;
    while (seconds < 10) {
		wdt_reset();		// Hold off Watchdog
    	if( settings.WatchSALUS ) {	
    		if( rf12_recvDone() )	{
    			if( (rf12_buf[0] == 212) /*&& (rf12_buf[1] >= 160) && (rf12_buf[1] != 255)*/ ) {
					Serial.print(elapsedSeconds);
					Serial.println("s Salus Packet");
        			Serial.flush();
            		for (byte i = 0; i < 15; i++) {
            			payload.messages[i] = rf12_buf[i];
                		showByte(rf12_buf[i]);
						printOneChar(' ');
           			 }
           			 payloadSize = BASIC_PAYLOAD_SIZE + 15;	// Forward Salus packet in next Jee packet

					Serial.print("\nSalus II Device:");  Serial.flush();
            		Serial.print(rf12_buf[1]);  // Device type
            		Serial.print(" Addr:");
            		payload.salusAddress = (rf12_buf[3] << 8) | rf12_buf[2];   // Guessing at a 16 bit address
            		Serial.print(payload.salusAddress);
            		Serial.print(" Command:");
            		payload.salusCommand = rf12_buf[4];
            		Serial.print(payload.salusCommand);

            		switch (rf12_buf[1]) {
                		case 165: // Thermostat
                    		payload.currentTemp = ((rf12_buf[6] << 8) | rf12_buf[5]);
                    		Serial.print(" Current=");  Serial.print(payload.currentTemp);
                    		payload.lowestTemp = ((rf12_buf[8] << 8) | rf12_buf[7]);
                    		Serial.print(" Lowest=");   Serial.print(payload.lowestTemp);
                    		payload.targetTemp = ((rf12_buf[10] << 8) | rf12_buf[9]);
                    		Serial.print(" Target=");   Serial.println(payload.targetTemp);
                    		payload.packetType = 2;      	// Indicate new data
							payloadReady = true;
							Serial.println("payload is ready");
							delay(3000);	// Wait for repeated Salus packets to pass
//							return;
                            break;
                            
                		case 166:   // OTO One Touch Override
                    		printOneChar(' ');
                    		Serial.print(rf12_buf[5]);
                    		if (getOTO) {
                        		settings.addrOTO = payload.salusAddress;
                        		settings.checkOTO = (rf12_buf[5] - rf12_buf[4]);
                        		Serial.print(" Learned OTO Offset ");  Serial.println(settings.checkOTO);  Serial.flush();
                        		getOTO = false; // Learn no more
                    		} else if ((payload.salusAddress == settings.addrOTO) && ((rf12_buf[5]) - rf12_buf[4] == settings.checkOTO)) {
                        		Serial.print(" One Touch Override Matched");
                        		Serial.println(", Relaying OTO");  Serial.flush();
                        		// Update checksum // byte commandOTO[] = {166, 163, 106, 0, 179};
                        		commandOTO[4] = (commandOTO[4] - commandOTO[3]) + rf12_buf[4];
                        		commandOTO[3] = rf12_buf[4];              // Copy over command byte
                        		//
                        		Serial.print(commandOTO[3]); printOneChar(':');  Serial.print(commandOTO[4]);  Serial.flush();
                        		rf12_sleep(RF12_WAKEUP);
                        		rf12_sendStart(0, &commandOTO, 5);               // Forward the OTO command.
                        		rf12_sendWait(1);                                // Wait for transmission complete.
                        		rf12_sleep(RF12_SLEEP);                                
                    		} else  {
                    			Serial.println(" One Touch Override Unknown");  
                    			Serial.flush();
                    		}
                    		break;
                    		
                		default:
                    		Serial.print(" Unknown ");
                    		for (byte i = 0; i < 8; i++) {;
                        		showByte(rf12_buf[i]);
                        		printOneChar(' ');
                    		}
                    		Serial.flush();
                    		break;
                		}
                		showStats();
                		Serial.println();  Serial.flush();
	        		} // rf12_buf
	    		} // rf12_recvDone
    		} // settings.WatchSALUS
    	} // while
    } // waitRF12

byte salusMode = false;
bool longBurn = true;

void loop () {
	Serial.print("Elapsed ");
	Serial.print(millis());
	Serial.print(" Next TX ");
	Serial.print(nextScheduled);
	Serial.print("-");
	Serial.println(elapsedSeconds);
	
/*
 * Setup to receive Salus transmissions
 */
	if (!(salusMode)) {
    	rf12_initialize (SALUSID, RF12_868MHZ, 212, SALUSFREQUENCY);// 868.3khz
    	rf12_sleep(RF12_SLEEP);                                     // Sleep while we tweak things
    	rf12_skip_hdr(2);                                           // Omit Jeelib header 2 bytes on transmission
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
    	rf12_control(0x9830 | (settings.salusTX & 0x07));           // 60khz freq shift, TX
#endif
        salusMode = true;
    	for (byte i = 0; i < 66; i++) rf12_buf[i] = 0;              // Clear buffer
    } //salusMode
                
	ds.reset();
    ds.skip();              // Next command to all devices
	ds.write(0x44);			// Start all temperature conversions.

	rf12_sleep(RF12_WAKEUP);
	waitRF12();	// Loop whilst checking radio
	Serial.println("Sleep radio"); Serial.flush();
	rf12_sleep(RF12_SLEEP);
	
	payload.ColdFeed = getTemp(ColdFeed);
	 Serial.print("Cold Feed:");
	 Serial.println(payload.ColdFeed);

	payload.BoilerFeed = getTemp(BoilerFeed);
//	boilerTrend = 0;
//	if (previousBoilerFeed) boilerTrend = payload.BoilerFeed - previousBoilerFeed;
//		previousBoilerFeed = payload.BoilerFeed;
         Serial.print("Boiler Feed:");
         Serial.print(payload.BoilerFeed);
         Serial.print(" trend:");
//         Serial.println(boilerTrend);
                        
        payload.CentralHeatingReturn = (getTemp(CentralHeatingReturn));
//        returnTrend = 0;
//        if (previousReturn) returnTrend = payload.CentralHeatingReturn - previousReturn;
//        previousReturn = payload.CentralHeatingReturn;
         Serial.print("Heating Return:");
         Serial.print(payload.CentralHeatingReturn);
         Serial.print(" trend:");
//         Serial.println(returnTrend);
        
		payload.HWTankTemp = getTemp(HWTankTemp);
 		Serial.print("HW Tank Temp:");
		Serial.println(payload.HWTankTemp);

		if (settings.tracking) {
		
			if (payload.currentTemp >= payload.targetTemp) {
				longBurn = false;	
			}
			
			if (delaySeconds <= elapsedSeconds) {
			
				if ((payload.BoilerFeed >= settings.maxBoiler) && 
			   	   ((payload.currentTemp + 50) >= payload.targetTemp)) {	   
					Serial.println("Above threshold && almost at target temperature");
					delaySeconds = elapsedSeconds;
					needSetback = true;
					
				} else
				if (payload.BoilerFeed < settings.maxBoiler) {
				 	Serial.println("Boiler below threshold");
					needSetback = false;
					delaySeconds = elapsedSeconds;
				}
				
				if ((payload.currentTemp + 50) <= payload.targetTemp) {			   
				 	Serial.println("0.5 degrees down");
					needSetback = false;
				 	if (!(longBurn)) {
				 		Serial.println("Long burn scheduled");
						delaySeconds = elapsedSeconds + burnTime;	// Stay on for 10mins
						longBurn = true;
					} else delaySeconds = elapsedSeconds;
				}
			}
			
		} else {
		
			needSetback = true;
			Serial.println("Tracking disabled");

		}	// settings.tracking		

		Serial.flush();

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

		checkSetback();
		
        Serial.println(backCount);
        Serial.println(payload.setBack);
		
        if ((elapsedSeconds >= nextScheduled) || (dataChanged)) {	// approx 60 seconds
        	dataChanged = false;
            payload.count++;
            if (NodeID = rf12_configSilent()) {
				rf12_control(0xC040);			// set low battery level to 2.2V
            	salusMode = false;
                Serial.print("Node ");
                Serial.print(NodeID);
                Serial.print(" sending packet #");
                Serial.print(payload.count);
                Serial.print(" length ");
                Serial.print(payloadSize);
                Serial.print(" sizeof ");
                Serial.println(sizeof (struct payload));
                Serial.flush();
            
                byte tries = sendACK();
        		nextScheduled = elapsedSeconds + minute;
            
                if (tries) { 
                     Serial.print(tries);
                     Serial.println(" attempt(s)");
                } else {
                     Serial.print("Packet #");
                     Serial.print(payload.count);
                     Serial.println(" Aborted");
                }
                Serial.flush();             
        } else {
            while( true ){
                rf12_sleep(RF12_SLEEP);
                Serial.println("RF12 eeprom not valid, run RF12Demo");  Serial.flush();
				delay(60000);
            }  
        }
    } 
                
	Serial.print("Voltage:");
    payload.voltage = readVcc();
    Serial.println(payload.voltage);
	if (payload.voltage > 28) {
    	 Serial.print("Looping ");
         Serial.println(++loopCount);
    	 Serial.flush();
    } else {
        rf12_sleep(RF12_SLEEP);
        Serial.println("Replace batteries");
    	Serial.flush();
    	cli();
    }
} // Loop
/*
 Deg  Code
 40.0 100.0   
 40.2 101.0   
 40.4 102.0   
 40.6 103.0   
 40.8 104.0   
 41.0 105.0   
 41.2 106.0   
 41.4 107.0   
 41.6 108.0   
 41.8 109.0   
 42.0 110.0   
 42.2 111.0   
 42.4 112.0   
 42.6 113.0   
 42.8 114.0   
 43.0 115.0   
 43.2 116.0   
 43.4 117.0   
 43.6 118.0   
 43.8 119.0   
 44.0 120.0   
 44.2 121.0   
 44.4 122.0   
 44.6 123.0   
 44.8 124.0   
 45.0 125.0   
 45.2 126.0   
 45.4 127.0   
 45.6 128.0   
 45.8 129.0   
 46.0 130.0   
 46.2 131.0   
 46.4 132.0   
 46.6 133.0   
 46.8 134.0   
 47.0 135.0   
 47.2 136.0   
 47.4 137.0   
 47.6 138.0   
 47.8 139.0   
 48.0 140.0   
 48.2 141.0   
 48.4 142.0   
 48.6 143.0   
 48.8 144.0   
 49.0 145.0   
 49.2 146.0   
 49.4 147.0   
 49.6 148.0   
 49.8 149.0   
 50.0 150.0   
 50.2 151.0   
 50.4 152.0   
 50.6 153.0   
 50.8 154.0   
 51.0 155.0   
 51.2 156.0   
 51.4 157.0   
 51.6 158.0   
 51.8 159.0   
 52.0 160.0   
 52.2 161.0   
 52.4 162.0   
 52.6 163.0   
 52.8 164.0   
 53.0 165.0   
 53.2 166.0   
 53.4 167.0   
 53.6 168.0   
 53.8 169.0   
 54.0 170.0   
 54.2 171.0   
 54.4 172.0   
 54.6 173.0   
 54.8 174.0   
 55.0 175.0   
 55.2 176.0   
 55.4 177.0   
 55.6 178.0   
 55.8 179.0   
 56.0 180.0   
 56.2 181.0   
 56.4 182.0   
 56.6 183.0   
 56.8 184.0   
 57.0 185.0   
 57.2 186.0   
 57.4 187.0   
 57.6 188.0   
 57.8 189.0   
 58.0 190.0   
 58.2 191.0   
 58.4 192.0   
 58.6 193.0   
 58.8 194.0   
 59.0 195.0   
 59.2 196.0   
 59.4 197.0   
 59.6 198.0   
 59.8 199.0   
*/
