/*
 AVA 70cms Tracker
 
 By Anthony Stirk M0UPU 
 
 October 2012 Version 3
 
 Thanks and credits :
 
 Interrupt Driven RTTY Code :
 Evolved from Rob Harrison's RTTY Code.
 Compare match register calculation by Phil Heron.
 Thanks to : http://www.engblaze.com/microcontroller-tutorial-avr-and-arduino-timer-interrupts/
 Suggestion to use Frequency Shift Registers by Dave Akerman (Daveake)/Richard Cresswell (Navrac)
 
 RFM22B Code from James Coxon http://ukhas.org.uk/guides:rfm22b 
 
 GPS Code from jonsowman and Joey flight computer CUSF
 https://github.com/cuspaceflight/joey-m/tree/master/firmware
 Big thanks to Dave Akerman!
 
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 See <http://www.gnu.org/licenses/>.
 */

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <util/crc16.h>
#include <SPI.h>
#include <RFM22.h>

#define RFM22B_PIN 10
#define RFM22B_SDN 3
#define BATTERYVOLTAGEPIN A0
#define ASCII 7          // ASCII 7 or 8
#define STOPBITS 2       // Either 1 or 2
#define TXDELAY 20        // Delay between sentence TX's
#define RTTY_BAUD 50     // Baud rate for use with RFM22B Max = 600
#define RADIO_FREQUENCY 434.200

uint8_t buf[60]; 
char txstring[80];
volatile int txstatus=1;
volatile int txstringlength=0;
volatile double txwarmup_ms=10000; // milliseconds for transmitter to warmup with steady carrrier before sending data
volatile char txc;
volatile int txi;
volatile int txj;
volatile int count=1;
uint8_t lock =0, sats = 0, hour = 0, minute = 0, second = 0;
int navmode = 0, battv=0, rawbattv=0, GPSerror = 0, lat_int=0,lon_int=0,txnulls=10;
int32_t lat = 0, lon = 0, alt = 0, maxalt = 0, lat_dec = 0, lon_dec =0, battvaverage=0;
int battvsmooth[5] , a;
boolean firstlock=1;


rfm22 radio1(RFM22B_PIN);

void setup() {

  analogReference(DEFAULT);
  pinMode(BATTERYVOLTAGEPIN,INPUT);
  Serial.begin(9600);
  setupRadio();
  setupGPS();
  initialise_interrupt();
}

void loop()
{



  //  if((lock==3) && (firstlock==1))
  //{
  // setupGPSpower();
  //firstlock=0;
  //rtty_txstring("PSM ENGAGED PSM ENGAGED\n");
  //}
  
  if (15 == txstatus) {  //It's time to transmit telemetry
  	gps_check_nav();
 	 if(navmode != 6) {
   		 gps_set_airborne();
  	 }
    prepare_data();
  if(alt>maxalt)
  {
    maxalt=alt;
  }
  	//Warm up radio transmitter
  	radio1.write(0x07, 0x08);//begin transmit carrier
  	delay(10000);
  	txstatus=2;
  }
  if (20 == txstatus)  {  //Radio shutdown
  	radio1.write(0x07, 0x00);//Set radio to standby low power mode.
  	txstatus=0;
  }
}   

void setupGPS() {
  Serial.println("$PUBX,40,GLL,0,0,0,0*5C");
  delay(1000);
  Serial.println("$PUBX,40,GGA,0,0,0,0*5A");
  delay(1000);
  Serial.println("$PUBX,40,GSA,0,0,0,0*4E");
  delay(1000);
  Serial.println("$PUBX,40,RMC,0,0,0,0*47");
  delay(1000);
  Serial.println("$PUBX,40,GSV,0,0,0,0*59");
  delay(1000);
  Serial.println("$PUBX,40,VTG,0,0,0,0*5E");
  delay(3000);
  gps_set_airborne();
}
void sendUBX(uint8_t *MSG, uint8_t len) {
  for(int i=0; i<len; i++) {
    Serial.write(MSG[i]);
  }
}

uint8_t gps_check_nav(void)
{
  uint8_t request[8] = {
    0xB5, 0x62, 0x06, 0x24, 0x00, 0x00,
    0x2A, 0x84                                                      };
  sendUBX(request, 8);

  // Get the message back from the GPS
  gps_get_data();

  // Verify sync and header bytes
  if( buf[0] != 0xB5 || buf[1] != 0x62 ){
    GPSerror = 41;
  }
  if( buf[2] != 0x06 || buf[3] != 0x24 ){
    GPSerror = 42;
  }
  // Check 40 bytes of message checksum
  if( !_gps_verify_checksum(&buf[2], 40) ) {
    GPSerror = 43;
  }

  // Return the navigation mode and let the caller analyse it
  navmode = buf[8];
}
void gps_get_data()
{
  Serial.flush();
  // Clear buf[i]
  for(int i = 0;i<60;i++) 
  {
    buf[i] = 0; // clearing buffer  
  }  
  int i = 0;
  unsigned long startTime = millis();

  while ((i<60) && ((millis() - startTime) < 1000) ) { 
    if (Serial.available()) {
      buf[i] = Serial.read();
      i++;
    }
  }
}
bool _gps_verify_checksum(uint8_t* data, uint8_t len)
{
  uint8_t a, b;
  gps_ubx_checksum(data, len, &a, &b);
  if( a != *(data + len) || b != *(data + len + 1))
    return false;
  else
    return true;
}
void gps_ubx_checksum(uint8_t* data, uint8_t len, uint8_t* cka,
uint8_t* ckb)
{
  *cka = 0;
  *ckb = 0;
  for( uint8_t i = 0; i < len; i++ )
  {
    *cka += *data;
    *ckb += *cka;
    data++;
  }
}
boolean getUBX_ACK(uint8_t *MSG) {
  uint8_t b;
  uint8_t ackByteID = 0;
  uint8_t ackPacket[10];
  unsigned long startTime = millis();

  // Construct the expected ACK packet    
  ackPacket[0] = 0xB5;	// header
  ackPacket[1] = 0x62;	// header
  ackPacket[2] = 0x05;	// class
  ackPacket[3] = 0x01;	// id
  ackPacket[4] = 0x02;	// length
  ackPacket[5] = 0x00;
  ackPacket[6] = MSG[2];	// ACK class
  ackPacket[7] = MSG[3];	// ACK id
  ackPacket[8] = 0;		// CK_A
  ackPacket[9] = 0;		// CK_B

  // Calculate the checksums
  for (uint8_t ubxi=2; ubxi<8; ubxi++) {
    ackPacket[8] = ackPacket[8] + ackPacket[ubxi];
    ackPacket[9] = ackPacket[9] + ackPacket[8];
  }

  while (1) {

    // Test for success
    if (ackByteID > 9) {
      // All packets in order!
      return true;
    }

    // Timeout if no valid response in 3 seconds
    if (millis() - startTime > 3000) { 
      return false;
    }

    // Make sure data is available to read
    if (Serial.available()) {
      b = Serial.read();

      // Check that bytes arrive in sequence as per expected ACK packet
      if (b == ackPacket[ackByteID]) { 
        ackByteID++;
      } 
      else {
        ackByteID = 0;	// Reset and look again, invalid order
      }

    }
  }
}
void gps_check_lock()
{
  GPSerror = 0;
  Serial.flush();
  // Construct the request to the GPS
  uint8_t request[8] = {
    0xB5, 0x62, 0x01, 0x06, 0x00, 0x00,
    0x07, 0x16                                                    };
  sendUBX(request, 8);

  // Get the message back from the GPS
  gps_get_data();
  // Verify the sync and header bits
  if( buf[0] != 0xB5 || buf[1] != 0x62 ) {
    GPSerror = 11;
  }
  if( buf[2] != 0x01 || buf[3] != 0x06 ) {
    GPSerror = 12;
  }

  // Check 60 bytes minus SYNC and CHECKSUM (4 bytes)
  if( !_gps_verify_checksum(&buf[2], 56) ) {
    GPSerror = 13;
  }

  if(GPSerror == 0){
    // Return the value if GPSfixOK is set in 'flags'
    if( buf[17] & 0x01 )
      lock = buf[16];
    else
      lock = 0;

    sats = buf[53];
  }
  else {
    lock = 0;
  }
}

void gps_set_airborne()
{
  int gps_set_sucess=0;
  uint8_t setNav[] = {
    0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC                                                      };
  while(!gps_set_sucess)
  {
    sendUBX(setNav, sizeof(setNav)/sizeof(uint8_t));
    gps_set_sucess=getUBX_ACK(setNav);
  }
}
void gps_get_position()
{
  GPSerror = 0;
  Serial.flush();
  // Request a NAV-POSLLH message from the GPS
  uint8_t request[8] = {
    0xB5, 0x62, 0x01, 0x02, 0x00, 0x00, 0x03,
    0x0A                                                };
  sendUBX(request, 8);

  // Get the message back from the GPS
  gps_get_data();

  // Verify the sync and header bits
  if( buf[0] != 0xB5 || buf[1] != 0x62 )
    GPSerror = 21;
  if( buf[2] != 0x01 || buf[3] != 0x02 )
    GPSerror = 22;

  if( !_gps_verify_checksum(&buf[2], 32) ) {
    GPSerror = 23;
  }

  if(GPSerror == 0) {
    // 4 bytes of longitude (1e-7)
    lon = (int32_t)buf[10] | (int32_t)buf[11] << 8 | 
      (int32_t)buf[12] << 16 | (int32_t)buf[13] << 24;

    lon_int=abs(lon/10000000);
    lon_dec=labs(lon) % 10000000;
    // 4 bytes of latitude (1e-7)
    lat = (int32_t)buf[14] | (int32_t)buf[15] << 8 | 
      (int32_t)buf[16] << 16 | (int32_t)buf[17] << 24;

    lat_int=abs(lat/10000000);
    lat_dec=labs(lat) % 10000000;


    // 4 bytes of altitude above MSL (mm)
    alt = (int32_t)buf[22] | (int32_t)buf[23] << 8 | 
      (int32_t)buf[24] << 16 | (int32_t)buf[25] << 24;
    alt /= 1000; // Correct to meters
  }

}
void gps_get_time()
{
  GPSerror = 0;
  Serial.flush();
  // Send a NAV-TIMEUTC message to the receiver
  uint8_t request[8] = {
    0xB5, 0x62, 0x01, 0x21, 0x00, 0x00,
    0x22, 0x67                                              };
  sendUBX(request, 8);

  // Get the message back from the GPS
  gps_get_data();

  // Verify the sync and header bits
  if( buf[0] != 0xB5 || buf[1] != 0x62 )
    GPSerror = 31;
  if( buf[2] != 0x01 || buf[3] != 0x21 )
    GPSerror = 32;

  if( !_gps_verify_checksum(&buf[2], 24) ) {
    GPSerror = 33;
  }

  if(GPSerror == 0) {
    if(hour > 23 || minute > 59 || second > 59)
    {
      GPSerror = 34;
    }
    else {
      hour = buf[22];
      minute = buf[23];
      second = buf[24];
    }
  }
}
uint16_t gps_CRC16_checksum (char *string)
{
  size_t i;
  uint16_t crc;
  uint8_t c;

  crc = 0xFFFF;

  // Calculate checksum ignoring the first two $s
  for (i = 5; i < strlen(string); i++)
  {
    c = string[i];
    crc = _crc_xmodem_update (crc, c);
  }

  return crc;
}

ISR(TIMER1_COMPA_vect)
{
  switch(txstatus) {
  case 0: // This is the optional delay between transmissions.
    txj++;
    if(txj>(TXDELAY*RTTY_BAUD)) { 
      txj=0;
      txstatus=1;
    }
    break;
  case 1: // Initialise transmission, take a copy of the string so it doesn't change mid transmission. 
    if(alt>maxalt)
    {
      maxalt=alt;
    }
    sprintf(txstring, "$$$$$K2VOL,%i,%02d:%02d:%02d,%s%i.%07ld,%s%i.%07ld,%ld,%d,%d,%i",count, hour, minute, second,lat < 0 ? "-" : "",lat_int,lat_dec,lon < 0 ? "-" : "",lon_int,lon_dec, maxalt,sats,lock,battvaverage);
    sprintf(txstring, "%s*%04X\n", txstring, gps_CRC16_checksum(txstring));
    maxalt=0;
    txstringlength=strlen(txstring);
    txstatus=15;
    txj=0;
    break;
  case 2: // Grab a char and lets go transmit it. 
    if ( txj < txstringlength)
    {
      txc = txstring[txj];
      txj++;
      txstatus=3;
      rtty_txbit (0); // Start Bit;
      txi=0;
    }
    else 
    {
      txstatus=20; // Should be finished, shut off radio
      txj=0;
      count++;
    }
    break;
  case 3:
    if(txi<ASCII)
    {
      txi++;
      if (txc & 1) rtty_txbit(1); 
      else rtty_txbit(0);	
      txc = txc >> 1;
      break;
    }
    else 
    {
      rtty_txbit (1); // Stop Bit
      txstatus=4;
      txi=0;
      break;
    } 
  case 4:
    if(STOPBITS==2)
    {
      rtty_txbit (1); // Stop Bit
      txstatus=2;
      break;
    }
    else
    {
      txstatus=2;
      break;
    }
  case 15:  //warming up radio, do nothing, this will be taken of outside of ISR
  		break;
  case 20:  //shut down radio taken care of outside of ISR.  
  		//External function will set txstatus=0;
  	break;

  }
}

void rtty_txbit (int bit)
{
  if (bit)
  {
    radio1.write(0x73,0x03); // High
  }
  else
  {
    radio1.write(0x73,0x00); // Low
  }
}

void setupRadio(){
  pinMode(RFM22B_SDN, OUTPUT);    // RFM22B SDN is on ARDUINO A3
  digitalWrite(RFM22B_SDN, LOW);
  delay(1000);
  rfm22::initSPI();
  radio1.init();
  radio1.write(0x71, 0x00); // unmodulated carrier
  //This sets up the GPIOs to automatically switch the antenna depending on Tx or Rx state, only needs to be done at start up
  radio1.write(0x0b,0x12);
  radio1.write(0x0c,0x15);
  radio1.setFrequency(RADIO_FREQUENCY);
  radio1.write(0x6D, 0x04);// turn tx low power 11db
  radio1.write(0x07, 0x08);//begin transmit carrier
  delay(1000);
  radio1.write(0x07, 0x00); //Enter Sleep Mode
  //radio1.write(0x07, 0x08);//begin transmit carrier
}

void setupGPSpower() {
  //Set GPS ot Power Save Mode
  uint8_t setCyclic[] = { 
    0xB5, 0x62, 0x06, 0x3B, 0x2C, 0x00, 0x01, 0x06, 0x00, 0x00, 0x00, 0x90, 0x02, 0x00, 0x10, 0x27             }; // Cyclic = 10s
  sendUBX(setCyclic, sizeof(setCyclic)/sizeof(uint8_t));
  uint8_t setPSM[] = { 
    0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x08, 0x01, 0x22, 0x92             }; // Setup for Power Save Mode (Default Cyclic 1s)

  sendUBX(setPSM, sizeof(setPSM)/sizeof(uint8_t));

}

void SetupGPSMaxpower() {
  //Set GPS for Max Performance Mode
  uint8_t setPSM[] = { 
    0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x08, 0x00, 0x21, 0x91             }; // Setup for Max Power Mode
  sendUBX(setPSM, sizeof(setPSM)/sizeof(uint8_t));
}

void prepare_data() {

  gps_check_lock();
  gps_get_position();
  gps_get_time();
  rawbattv = analogRead(BATTERYVOLTAGEPIN);
  battv = rawbattv*41;

  battvsmooth[4] = battvsmooth[3];
  battvsmooth[3] = battvsmooth[2];
  battvsmooth[2] = battvsmooth[1];
  battvsmooth[1] = battvsmooth[0];
  battvsmooth[0] = battv;
  battvaverage = (battvsmooth[0]+battvsmooth[1]+ battvsmooth[2]+battvsmooth[3]+battvsmooth[4])/5;

}
void initialise_interrupt() 
{
  // initialize Timer1
  cli();          // disable global interrupts
  TCCR1A = 0;     // set entire TCCR1A register to 0
  TCCR1B = 0;     // same for TCCR1B
  OCR1A = F_CPU / 1024 / RTTY_BAUD - 1;  // set compare match register to desired timer count:
  TCCR1B |= (1 << WGM12);   // turn on CTC mode:
  // Set CS10 and CS12 bits for:
  TCCR1B |= (1 << CS10);
  TCCR1B |= (1 << CS12);
  // enable timer compare interrupt:
  TIMSK1 |= (1 << OCIE1A);
  sei();          // enable global interrupts
}



