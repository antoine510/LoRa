#ifndef LORA_H
#define LORA_H

#include <Arduino.h>

// uncomment for debug output
#define DEBUG

/** RFM bandwidth */
enum rfm_bandwidth : uint8_t {
	BW7_8 = 0x0,
	BW10_4 = 0x1,
	BW15_6 = 0x2,
	BW20_8 = 0x3,
	BW31_25 = 0x4,
	BW41_7 = 0x5,
	BW62_5 = 0x6,
	BW125 = 0x7,
	BW250 = 0x8,
	BW500 = 0x9
};

/** RFM coding rate */
enum rfm_coding_rate : uint8_t {
	CR4_5 = 0x1,
	CR4_6 = 0x2,
	CR4_7 = 0x3,
	CR4_8 = 0x4
};


#define RFM9x_VER 0x12 ///< Expected RFM9x RegVersion

#define OP_MODE_SLEEP 0x80      ///< LoRa mode, no FSK registers, high-freq mode, SLEEP
#define OP_MODE_STDBY 0x81      ///< LoRa mode, no FSK registers, high-freq mode, STDBY
#define OP_MODE_TX 0x83      ///< LoRa mode, no FSK registers, high-freq mode, TX

#define DIO0_TXDONE 0x40		///< DIO pin 0 gets TxDone interrupt

#define FIFO_TX_ADDR 0x80		///< TX start address in FIFO buffer

/* RFM Registers */
#define REG_FIFO 0x00			///< FIFO access register
#define REG_OP_MODE 0x01      ///< Operational mode of module
#define REG_FRF_MSB 0x06      ///< RF Carrier Frequency MSB
#define REG_FRF_MID 0x07      ///< RF Carrier Frequency Intermediate
#define REG_FRF_LSB 0x08      ///< RF Carrier Frequency LSB
#define REG_PA_CONFIG 0x09    ///< PA selection and Output Power control
#define REG_FIFO_ADDR_PTR 0x0D		///< FIFO address pointer for SPI
#define REG_MODEM_CONFIG_1 0x1D     ///< Modem configuration register 1
#define REG_MODEM_CONFIG_2 0x1E     ///< Modem configuration register 2
#define REG_PAYLOAD_LENGTH 0x22		///< Payload length register
#define REG_DIO_MAPPING_1 0x40		///< Digital I/O mapping
#define REG_VER 0x42          ///< RFM9x version register

class LoRa {
public:
	LoRa(char rfm_dio0, char rfm_cs, char rfm_rst);

	void setFrequency(uint32_t freq);
	void setBandwidth(enum rfm_bandwidth bandwidth);
	void setCodingRate(enum rfm_coding_rate coding_rate);
	void setTXPower(uint8_t power);
	void setSpreadingFactor(uint8_t sf);

	bool begin(void);

	void sendData(uint8_t* data, uint8_t len);

private:
	char _pin_dio0, _pin_cs, _pin_rst;
	enum rfm_bandwidth _bandwidth;
	enum rfm_coding_rate _codingRate;
	unsigned char _spreadingFactor;
	unsigned char _freqMSB, _freqMID, _freqLSB;
	uint8_t _txPower;

	void writeRaw(uint8_t addr, uint8_t byte);
	bool writeChecked(uint8_t addr, uint8_t byte);
	uint8_t read(uint8_t addr);
};

#endif
