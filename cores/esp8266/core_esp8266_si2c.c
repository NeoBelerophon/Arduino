/*
  si2c.c - Software I2C library for esp8266

  Copyright (c) 2015 Hristo Gochkov. All rights reserved.
  This file is part of the esp8266 core for Arduino environment.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/
#include "twi.h"
#include "pins_arduino.h"
#include "wiring_private.h"

unsigned char twi_dcount = 18;
static unsigned char twi_sda, twi_scl;
static unsigned char twi_slaveAddress;
static unsigned char twi_state;
static unsigned char twi_receivePos;


static void (*twi_onSlaveTransmit)(void);
static void (*twi_onSlaveReceive)(uint8_t*, int);

static uint8_t twi_masterBuffer[TWI_BUFFER_LENGTH];
static volatile uint8_t twi_masterBufferIndex;
static volatile uint8_t twi_masterBufferLength;

static uint8_t twi_txBuffer[TWI_BUFFER_LENGTH];
static volatile uint8_t twi_txBufferIndex;
static volatile uint8_t twi_txBufferLength;

static uint8_t twi_rxBuffer[TWI_BUFFER_LENGTH];
static volatile uint8_t twi_rxBufferIndex;


#define SDA_LOW()   (GPES = (1 << twi_sda)) //Enable SDA (becomes output and since GPO is 0 for the pin, it will pull the line low)
#define SDA_HIGH()  (GPEC = (1 << twi_sda)) //Disable SDA (becomes input and since it has pullup it will go high)
#define SDA_READ()  ((GPI & (1 << twi_sda)) != 0)
#define SCL_LOW()   (GPES = (1 << twi_scl))
#define SCL_HIGH()  (GPEC = (1 << twi_scl))
#define SCL_READ()  ((GPI & (1 << twi_scl)) != 0)


/*
 * R/~W bit in SLA+R/W address field.
 */
#define TW_READ         1
#define TW_WRITE  0

#ifndef FCPU80
#define FCPU80 80000000L
#endif

#if F_CPU == FCPU80
#define TWI_CLOCK_STRETCH 800
#else
#define TWI_CLOCK_STRETCH 1600
#endif

void twi_setClock(unsigned int freq){
#if F_CPU == FCPU80
  if(freq <= 100000) twi_dcount = 19;//about 100KHz
  else if(freq <= 200000) twi_dcount = 8;//about 200KHz
  else if(freq <= 300000) twi_dcount = 3;//about 300KHz
  else if(freq <= 400000) twi_dcount = 1;//about 400KHz
  else twi_dcount = 1;//about 400KHz
#else
  if(freq <= 100000) twi_dcount = 32;//about 100KHz
  else if(freq <= 200000) twi_dcount = 14;//about 200KHz
  else if(freq <= 300000) twi_dcount = 8;//about 300KHz
  else if(freq <= 400000) twi_dcount = 5;//about 400KHz
  else if(freq <= 500000) twi_dcount = 3;//about 500KHz
  else if(freq <= 600000) twi_dcount = 2;//about 600KHz
  else twi_dcount = 1;//about 700KHz
#endif
}

void twi_init(unsigned char sda, unsigned char scl){
  twi_sda = sda;
  twi_scl = scl;
  pinMode(twi_sda, INPUT_PULLUP);
  pinMode(twi_scl, INPUT_PULLUP);
  twi_setClock(100000);
}

void twi_stop(void){
  pinMode(twi_sda, INPUT);
  pinMode(twi_scl, INPUT);
}

static void twi_delay(unsigned char v){
  unsigned int i;
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
  unsigned int reg;
  for(i=0;i<v;i++) reg = GPI;
#pragma GCC diagnostic pop
}

static bool twi_write_start(void) {
  SCL_HIGH();
  SDA_HIGH();
  if (SDA_READ() == 0) return false;
  twi_delay(twi_dcount);
  SDA_LOW();
  twi_delay(twi_dcount);
  return true;
}

static bool twi_write_stop(void){
  unsigned int i = 0;
  SCL_LOW();
  SDA_LOW();
  twi_delay(twi_dcount);
  SCL_HIGH();
  while (SCL_READ() == 0 && (i++) < TWI_CLOCK_STRETCH);// Clock stretching (up to 100us)
  twi_delay(twi_dcount);
  SDA_HIGH();
  twi_delay(twi_dcount);

  return true;
}

static bool twi_write_bit(bool bit) {
  unsigned int i = 0;
  SCL_LOW();
  if (bit) SDA_HIGH();
  else SDA_LOW();
  twi_delay(twi_dcount+1);
  SCL_HIGH();
  while (SCL_READ() == 0 && (i++) < TWI_CLOCK_STRETCH);// Clock stretching (up to 100us)
  twi_delay(twi_dcount);
  return true;
}

static bool twi_read_bit(void) {
  unsigned int i = 0;
  SCL_LOW();
  SDA_HIGH();
  twi_delay(twi_dcount+2);
  SCL_HIGH();
  while (SCL_READ() == 0 && (i++) < TWI_CLOCK_STRETCH);// Clock stretching (up to 100us)
  bool bit = SDA_READ();
  twi_delay(twi_dcount);
  return bit;
}

static bool twi_write_byte(unsigned char byte) {
  unsigned char bit;
  for (bit = 0; bit < 8; bit++) {
    twi_write_bit(byte & 0x80);
    byte <<= 1;
  }
  return !twi_read_bit();//NACK/ACK
}

static unsigned char twi_read_byte(bool nack) {
  unsigned char byte = 0;
  unsigned char bit;
  for (bit = 0; bit < 8; bit++) byte = (byte << 1) | twi_read_bit();
  twi_write_bit(nack);
  return byte;
}

unsigned char twi_writeTo(unsigned char address, unsigned char * buf, unsigned int len, unsigned char sendStop){
  unsigned int i;
  if(!twi_write_start()) return 4;//line busy
  if(!twi_write_byte(((address << 1) | 0) & 0xFF)) {
    if (sendStop) twi_write_stop();
    return 2; //received NACK on transmit of address
  }
  for(i=0; i<len; i++) {
    if(!twi_write_byte(buf[i])) {
      if (sendStop) twi_write_stop();
      return 3;//received NACK on transmit of data
    }
  }
  if(sendStop) twi_write_stop();
  i = 0;
  while(SDA_READ() == 0 && (i++) < 10){
    SCL_LOW();
    twi_delay(twi_dcount);
    SCL_HIGH();
    twi_delay(twi_dcount);
  }
  return 0;
}

unsigned char twi_readFrom(unsigned char address, unsigned char* buf, unsigned int len, unsigned char sendStop){
  unsigned int i;
  if(!twi_write_start()) return 4;//line busy
  if(!twi_write_byte(((address << 1) | 1) & 0xFF)) {
    if (sendStop) twi_write_stop();
    return 2;//received NACK on transmit of address
  }
  for(i=0; i<(len-1); i++) buf[i] = twi_read_byte(false);
  buf[len-1] = twi_read_byte(true);
  if(sendStop) twi_write_stop();
  i = 0;
  while(SDA_READ() == 0 && (i++) < 10){
    SCL_LOW();
    twi_delay(twi_dcount);
    SCL_HIGH();
    twi_delay(twi_dcount);
  }
  return 0;
}



void twi_isr(void){
	uint8_t value = SDA_READ();
	switch(twi_state){
	case 0: // wait for start condition - SCL & SDA low
		if(!value) {
			twi_state++;
			twi_receivePos = 0;
		}
		break;
	case 1: // receive address
		if(twi_receivePos < 8){
			twi_masterBuffer[0] &= value;
			twi_masterBuffer[0] <<= 1;
			twi_receivePos++;
		} else {
			if(twi_masterBuffer[0] == twi_slaveAddress) // if this message is for us send ACK
			{
				SDA_LOW(); // send ACK

				if (twi_masterBuffer[0] & TW_WRITE){ // transmit
					if(twi_onSlaveTransmit != NULL) // notify callback if connected
						twi_onSlaveTransmit();
					twi_state = 2;
				} else { // receive
					twi_state = 3;
				}
				twi_receivePos = 0;

			} else { // don't care
				twi_state = 0;
			}
		}
		break;
	case 2: // transmit
		if(twi_txBufferIndex < twi_txBufferLength){
			if(twi_receivePos < 8)
			{
				if(twi_txBuffer[twi_txBufferIndex] & 0x80 > 0)
				{
					SDA_HIGH();
				} else {
					SDA_LOW();
				}
				twi_txBuffer[twi_txBufferIndex] <<= 1;

				if(twi_receivePos == 7)
					twi_txBufferIndex++;
			}
		} else {
			twi_state = 4;
		}
		break;
	case 3: // receive
		if(twi_receivePos < 8){
			twi_rxBuffer[0] &= value;
			twi_rxBuffer[0] <<= 1;
			twi_receivePos++;
		} else {
			if(twi_onSlaveReceive != NULL)
				twi_onSlaveReceive(twi_rxBuffer, 1);
		}
		twi_rxBufferIndex = 0;
		twi_state = 4;
		break;
	case 4: // ACK
		SDA_LOW();
		twi_state = 0;
	}
}

void twi_setAddress(uint8_t address){
	twi_slaveAddress = address << 1;
	twi_state = 0;
	attachInterrupt(twi_scl, twi_isr, FALLING);
}

/*
 * Function twi_attachSlaveRxEvent
 * Desc     sets function called before a slave read operation
 * Input    function: callback function to use
 * Output   none
 */
void twi_attachSlaveRxEvent( void (*function)(uint8_t*, int) )
{
  twi_onSlaveReceive = function;
}

/*
 * Function twi_attachSlaveTxEvent
 * Desc     sets function called before a slave write operation
 * Input    function: callback function to use
 * Output   none
 */
void twi_attachSlaveTxEvent( void (*function)(void) )
{
  twi_onSlaveTransmit = function;
}

/*
 * Function twi_transmit
 * Desc     fills slave tx buffer with data
 *          must be called in slave tx event callback
 * Input    data: pointer to byte array
 *          length: number of bytes in array
 * Output   1 length too long for buffer
 *          2 not slave transmitter
 *          0 ok
 */
uint8_t twi_transmit(const uint8_t* data, uint8_t length)
{
  uint8_t i;

  // ensure data will fit into buffer
  if(TWI_BUFFER_LENGTH < length){
    return 1;
  }

  // ensure we are currently a slave transmitter
  //if(TWI_STX != twi_state){
  //  return 2;
  //}

  // set length and copy data into tx buffer
  twi_txBufferLength = length;
  for(i = 0; i < length; ++i){
    twi_txBuffer[i] = data[i];
  }

  return 0;
}
