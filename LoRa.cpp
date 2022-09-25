#include "LoRa.h"
#include <SPI.h>

#ifdef DEBUG
#define write(a, d) writeChecked(a, d)
#else
#define write(a, d) writeRaw(a, d)
#endif

SPISettings spiSet(4000000, MSBFIRST, SPI_MODE0);

/*!
	@brief Sets the RFM frequency.
	@param freq Which frequency to use as a multiple of 61.035 Hz
*/
void LoRa::setFrequency(uint32_t freq) {
	_freqMSB = (freq >> 16) & 0xff;
	_freqMID = (freq >> 8) & 0xff;
	_freqLSB = freq & 0xff;
}

/*!
	@brief Sets the RFM bandwidth.
	@param bandwidth Which bandwidth to use in kHz
*/
void LoRa::setBandwidth(enum rfm_bandwidth bandwidth) {
	_bandwidth = bandwidth;
}

/*!
	@brief Sets the RFM coding rate.
	@param coding_rate Which coding rate to use
*/
void LoRa::setCodingRate(enum rfm_coding_rate coding_rate) {
	_codingRate = coding_rate;
}

/*!
	@brief Sets the RFM spreading factor.
	@param sf Which spreading factor to use
*/
void LoRa::setSpreadingFactor(uint8_t sf) {
	if(sf < 6 || sf > 12) return;
	_spreadingFactor = sf;
}

/** Set the transmit power in dBm 0 to 17 */
void LoRa::setTXPower(uint8_t power) {
	bool PaBoost = false;
	uint8_t OutputPower = 10;		// 0-15
	uint8_t MaxPower = 7;		// 0-7

	if(power <= 13) {
		PaBoost = false;
		MaxPower = 4;		// 13.2 dBm
		OutputPower = power + 2;
	} else if(power <= 17) {
		PaBoost = true;
		OutputPower = power - 2;
	}

	// Pack the above data to one byte and send it
	_txPower = (PaBoost << 7) + (MaxPower << 4) + OutputPower;
}

/*!
	@brief  Instanciates a new LoRa class, including assigning
			irq and cs pins to the RFM breakout.
	@param    rfm_dio0
			  The RFM module's DIO0 pin.
	@param    rfm_cs
			  The RFM module's chip select pin (NSS).
	@param    rfm_rst
			  The RFM module's reset pin.
*/
LoRa::LoRa(char rfm_dio0, char rfm_cs, char rfm_rst) {
	_pin_dio0 = rfm_dio0;
	_pin_cs = rfm_cs;
	_pin_rst = rfm_rst;
}


/*!
	@brief  Initializes the RFM, including configuring SPI, configuring
			the frameCounter and txrandomNum.
	@return True if the RFM has been initialized
*/
bool LoRa::begin() {
	SPI.begin();

	pinMode(_pin_cs, OUTPUT);
	pinMode(_pin_dio0, INPUT);
	pinMode(_pin_rst, OUTPUT);
	pinMode(11, OUTPUT);	// SPI MOSI set as output
	pinMode(12, INPUT);		// SPI MISO set as input
	Serial.println("Pinmode set");

	// Reset the RFM radio module
	digitalWrite(_pin_rst, LOW);
	delayMicroseconds(100);
	digitalWrite(_pin_rst, HIGH);
	delay(100);
	Serial.println("RFM95W reset");

	delay(100);

	uint8_t ver = read(REG_VER);
	Serial.println("Board version is " + String(ver));

	delay(100);
		

	// Switch RFM to sleep and LoRa, LoRa mode request is ignored the first time so repeat
	write(REG_OP_MODE, OP_MODE_SLEEP);
	write(REG_OP_MODE, OP_MODE_SLEEP);

	// Switch DIO0 to TxDone
	write(REG_DIO_MAPPING_1, DIO0_TXDONE);

	// select rfm channel
	write(REG_FRF_MSB, _freqMSB);
	write(REG_FRF_MID, _freqMID);
	write(REG_FRF_LSB, _freqLSB);

	// Implicit header mode & payload CRC on: set bit 1 of config_1 and bit 3 of config_2
	unsigned char modem_config_1 = ((unsigned char)_bandwidth << 4) + ((unsigned char)_codingRate << 1) + 1;
	unsigned char modem_config_2 = (_spreadingFactor << 4) + (1 << 2);

	// Set RFM configuration
	write(REG_MODEM_CONFIG_1, modem_config_1);
	write(REG_MODEM_CONFIG_2, modem_config_2);

	write(REG_PA_CONFIG, _txPower);

	return 1;
}

void LoRa::sendData(uint8_t* data, uint8_t len) {
	// Set RFM in Standby mode wait on mode ready
	write(REG_OP_MODE, OP_MODE_STDBY);

	// Set payload length to the right length
	write(REG_PAYLOAD_LENGTH, len);

	// Set SPI pointer to start of Tx part in FiFo
	write(REG_FIFO_ADDR_PTR, FIFO_TX_ADDR);

	// Write Payload to FiFo
	for(uint8_t i = 0; i < len; ++i) {
		writeRaw(REG_FIFO, data[i]);
	}
	// Switch RFM to Tx
	write(REG_OP_MODE, OP_MODE_TX);

	// Wait _irq to pull high
	while(digitalRead(_pin_dio0) == LOW) {}

	// Switch RFM to sleep
	write(REG_OP_MODE, OP_MODE_SLEEP);
}

void LoRa::writeRaw(uint8_t addr, uint8_t byte) {
	SPI.beginTransaction(spiSet);

	digitalWrite(_pin_cs, LOW);
	delayMicroseconds(10);
	SPI.transfer(addr | 0x80);
	SPI.transfer(byte);
	delayMicroseconds(10);
	digitalWrite(_pin_cs, HIGH);
	
	SPI.endTransaction();

	delay(1);

	//Serial.println("Wrote address " + String(addr) + " : " + String(byte));
}

bool LoRa::writeChecked(uint8_t addr, uint8_t byte) {
	writeRaw(addr, byte);
	if(read(addr) != byte) {
		Serial.println("Could not write address " + String(addr));
		return false;
	}
	return true;
}

uint8_t LoRa::read(uint8_t addr) {
	SPI.beginTransaction(spiSet);
	digitalWrite(_pin_cs, LOW);
	delayMicroseconds(10);
	SPI.transfer(addr);
	uint8_t byte = (uint8_t)SPI.transfer(0x00);
	delayMicroseconds(10);
	digitalWrite(_pin_cs, HIGH);
	SPI.endTransaction();

	//Serial.println("Read address " + String(addr) + " : " + String(byte));

	return byte;
}

