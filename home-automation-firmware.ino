#include <EEPROM.h>
#include "STM32_CAN.h"
#include "STM32F103Rx.h"

// Firmware
// Version is defined as 00 <Mayor> <Minor> <Bugfix>
#define FIRMWARE_VERSION  0x00000100UL

// CommCtrl Byte
#define MESSAGE_TYPE_BIT  0x80
#define DISCOVERY_BIT     0x40
#define PING_BIT          0x20
#define ACK_BIT           0x10
#define ERROR_BIT         0x08

// DataCtrl Byte
#define DATA_CONFIG_BIT     0x80
#define DATA_EEPROM_BIT     0x40
#define DATA_OPERATION_BIT  0x20
#define DATA_SIGNAL_BIT     0x10
#define DATA_DIRECTION_BIT  0x08
#define DATA_TYPE_BIT       0x06

// ConfCtrl Byte Enums
#define CONF_BITS                  0b00011111
#define CONF_BUTTON_RISING_EDGE    0b00000000
#define CONF_BUTTON_FALLIN_EDGE    0b00000001
#define CONF_SWITCH                0b00000010
#define CONF_ACTION_TOGGLE         0b00000011
#define CONF_ACTION_HIGH           0b00000100
#define CONF_ACTION_LOW            0b00000101
#define CONF_DEBOUNCE              0b00000110
#define CONF_LONGPRESS             0b00000111
#define CONF_LONGPRESS_DELAYOFF    0b00001000
#define CONF_BYPASS_INSTANTLY      0b00001001
#define CONF_BYPASS_ON_DIP_SWITCH  0b00001010
#define CONF_BYPASS_ON_DISCONNECT  0b00001011

// Protocol types
#define TYPE_DIGITAL 0
#define TYPE_ANALOG  1
#define TYPE_OUTPUT  0
#define TYPE_INPUT   1
#define TYPE_BIT     0b00 // 00 = Bit
#define TYPE_BYTE    0b01 // 01 = Byte (8-bit)
#define TYPE_INT     0b10 // 10 = Integer (32-bit)
#define TYPE_FLOAT   0b11 // 11 = Float

// Sizes
#define SIZE_DEVICE_ADDRESS  5
#define SIZE_INPUT_DIGITAL   16
#define SIZE_INPUT_ANALOG    4
#define SIZE_OUTPUT_DIGITAL  12

// Error codes (packed in Data on error/ack)
#define ERR_UNKNOWN               0x00000001UL
#define ERR_OPERATION_NOT_ALLOWED 0x00000002UL
#define ERR_CONFIG_NOT_ALLOWED    0x00000003UL
#define ERR_INVALID_TYPE          0x00000004UL
#define ERR_INVALID_PORT          0x00000005UL

// CAN setup
STM32_CAN Can1(CAN_RX, CAN_TX);

#define CAN_BASE_ADDRESS 0x000
#define CAN_BCAST_ADDRES 0x7FF // broadcast frame receiver

// Logical device identity
uint8_t deviceId = 0;

// Hardware pin maps (from your original sketch)
const int deviceAddressPins[] = { DEV_A1, DEV_A2, DEV_A3, DEV_A4, DEV_A5 };
const int inputDigitalPins[]  = { DI_01, DI_02, DI_03, DI_04, DI_05, DI_06, DI_07, DI_08, DI_09, DI_10, DI_11, DI_12, DI_13, DI_14, DI_15, DI_16 };
const int outputDigitalPins[] = { DO_01, DO_02, DO_03, DO_04, DO_05, DO_06, DO_07, DO_08, DO_09, DO_10, DO_11, DO_12 };
const int configurationPins[] = { C_01, C_02 };


struct InputDigital {
	uint8_t pin;
	uint8_t value;
	int32_t debounce;
	int32_t pressedTime;
};
InputDigital inputDigitals[SIZE_INPUT_DIGITAL];

struct OutputDigital {
	uint8_t pin;
	uint8_t value;
	int64_t delayOff;
};
OutputDigital outputDigitals[SIZE_OUTPUT_DIGITAL];

// Structure for the input config
struct ConfigRegister {
	int32_t version;
	bool isButtonRisingEdge;
	bool isButtonFallingEdge;
	bool isSwitch;
	int32_t actionToggle; // Toggle output ports in bitmap
	int32_t actionHigh; // Switch on output ports in bitmap
	int32_t actionLow; // Switch off output ports in bitmap
	int32_t debounce; // Debounce in microseconds
	int32_t longpress; // Debounce in microseconds
	int32_t longpressDelayOff; // Debounce in microseconds
	bool bypassInstantly;
	bool bypassOnDIPSwitch;
	int32_t bypassOnDisconnect; // bypess after x miliseconds from last ping
};

// Set configuration for each input pin
ConfigRegister inputConfig[SIZE_INPUT_DIGITAL] = {0};

// Global variables
int32_t loopTimeDiff = 0;
uint64_t loopTimeLast = 0;

// Last time in milliseconds
int32_t lastSyncRemote = 0;

// State of the DIP switch C_02
bool dipSwitchBypass = false;


// Helpers to pack 32-bit data (MSB..LSB)
static inline void u32ToBytes(uint32_t source, uint8_t* target) {
	target[0] = (uint8_t)(source >> 24);
	target[1] = (uint8_t)(source >> 16);
	target[2] = (uint8_t)(source >> 8);
	target[3] = (uint8_t)(source);
}

// Compute Device ID from DIP
static inline uint8_t computeDeviceAddress() {
	uint8_t id = 0;
	for (int i = 0; i < SIZE_DEVICE_ADDRESS; i++) {
		pinMode(deviceAddressPins[i], INPUT_PULLUP);
		// LOW means switch ON -> bit=1
		id |= ((digitalRead(deviceAddressPins[i]) == LOW) ? 1 : 0) << i;
	}
	return id;
}

// Send CAN frame
void canWriteFrame(uint16_t to, uint8_t from, uint8_t commCtrl, uint8_t dataCtrl, uint8_t port, uint32_t data) {
	CAN_message_t tx{};
	tx.id  = (uint16_t)(CAN_BASE_ADDRESS + (to & 0xFF));
	tx.len = 8;

	tx.buf[0] = from;             // B1 From
	tx.buf[1] = commCtrl;         // B2 CommCtrl
	tx.buf[2] = dataCtrl;         // B3 DataCtrl
	tx.buf[3] = port;             // B4 Port
	u32ToBytes(data, &tx.buf[4]); // B5..B8 Data MSB..LSB

	Can1.write(tx);
}

// Send acknowledge frame
void sendAck(uint8_t to, uint8_t from, uint8_t commCtrl, uint8_t dataCtrl, uint8_t port, uint32_t data) {
	// Set A=1, keep R=1, O/C etc. as mirrored from request (your spec)
	uint8_t cc = (commCtrl | ACK_BIT) & ~ERROR_BIT; // ensure E=0
	canWriteFrame(to, from, cc, dataCtrl, port, data);
}

// Send error frame
void sendError(uint8_t to, uint8_t from, uint8_t commCtrl, uint8_t dataCtrl, uint8_t port, uint32_t errCode) {
	uint8_t cc = (commCtrl | ACK_BIT | ERROR_BIT);
	canWriteFrame(to, from, cc, dataCtrl, port, errCode);
}

uint32_t bitmapOutputDigitals() {
	// 12 outputs → pack into lower bits of 32-bit
	uint32_t data = 0;
	for (int i = 0; i < SIZE_OUTPUT_DIGITAL; i++) {
		data |= ((outputDigitals[i].value & 0x01) ? 1UL : 0UL) << i;
	}
	return data;
}

uint32_t bitmapInputDigitals() {
	// 16 inputs → pack into lower 16 bits
	uint32_t data = 0;
	for (int i = 0; i < SIZE_INPUT_DIGITAL; i++) {
		data |= ((inputDigitals[i].value & 0x01) ? 1UL : 0UL) << i;
	}
	return data;
}

// Change status of the output port
void setDigitalOutput(uint8_t pin, uint8_t value, uint32_t setDelayOff) {
	if (pin < SIZE_OUTPUT_DIGITAL) {
		digitalWrite(outputDigitalPins[pin], value);
		outputDigitals[pin].value = value;
		if (value == LOW) {
			// Reset delayOff
			outputDigitals[pin].delayOff = 0;
		} else {
			// Set delayOff
			outputDigitals[pin].delayOff = setDelayOff * 1000;
		}
	}
}

void resetDigitalInputConf(uint8_t port) {
	inputConfig[port].version             = FIRMWARE_VERSION;
	inputConfig[port].isButtonRisingEdge  = false;
	inputConfig[port].isButtonFallingEdge = false;
	inputConfig[port].isSwitch            = false;
	inputConfig[port].actionToggle        = 0;
	inputConfig[port].actionHigh          = 0;
	inputConfig[port].actionLow           = 0;
	inputConfig[port].debounce            = 0;
	inputConfig[port].longpress           = 0;
	inputConfig[port].longpressDelayOff   = 0;
	inputConfig[port].bypassInstantly     = false;
	inputConfig[port].bypassOnDIPSwitch   = false;
	inputConfig[port].bypassOnDisconnect  = 0;
}

// Handle one received CAN frame for us
void canProcessFrame(const CAN_message_t& rx) {
	// Only process frames where CAN ID matches us or broadcast
	if (rx.id != (uint16_t)(CAN_BASE_ADDRESS + deviceId) && rx.id != CAN_BCAST_ADDRES) return;
	if (rx.len != 8) return;

	// Unpack payload
	uint8_t from     = rx.buf[0];        // B1
	uint8_t commCtrl = rx.buf[1];        // B2
	uint8_t dataCtrl = rx.buf[2];        // B3
	uint8_t port     = rx.buf[3];        // B4
	uint32_t data    = 0x00;
	uint8_t conf     = 0x00;

	// Communication control parameters
	bool isCommand     = (commCtrl & MESSAGE_TYPE_BIT) >> 7;
	bool isDiscovery   = (commCtrl & DISCOVERY_BIT) >> 6;
	bool isPing        = (commCtrl & PING_BIT) >> 5;
	bool isAcknowledge = (commCtrl & ACK_BIT) >> 4;
	bool isError       = (commCtrl & ERROR_BIT) >> 3;
	
	// Data control parameters
	bool isConfig      = (dataCtrl & DATA_CONFIG_BIT) >> 7;
	bool isWriteEEPROM = (dataCtrl & DATA_EEPROM_BIT) >> 6;
	bool isAnalog      = (dataCtrl & DATA_SIGNAL_BIT) >> 5;
	bool isWrite       = (dataCtrl & DATA_OPERATION_BIT) >> 4;
	bool isInput       = (dataCtrl & DATA_DIRECTION_BIT) >> 3;
	uint8_t dataType   = (dataCtrl & DATA_TYPE_BIT) >> 1;

	// Discovery
	// Return device address when asked to identify
	if (rx.id == CAN_BCAST_ADDRES && isDiscovery && !isWrite && !isConfig) {
		sendAck(from, deviceId, DISCOVERY_BIT | ACK_BIT, TYPE_INT << 2, 0, FIRMWARE_VERSION);
		return;
	}

	// Ping - pong
	if ((rx.id == (uint16_t)(CAN_BASE_ADDRESS + deviceId) || rx.id == CAN_BCAST_ADDRES) && isPing) {
		lastSyncRemote = millis();
		sendAck(from, deviceId, commCtrl | ACK_BIT, dataCtrl, 0, 0);
		return;
	}

	// Only reply to the broadcast address on discovery or ping messages
	if (rx.id == CAN_BCAST_ADDRES ) {
		// TODO Error numbers
		sendError(from, deviceId, commCtrl, dataCtrl, port, ERR_UNKNOWN);
		return;
	}

	// Config
	if (isCommand && isConfig && !isAnalog) {
		// Write current configuration to EEPROM
		if (isWriteEEPROM) {
			EEPROM.put(0, inputConfig);
			sendAck(from, deviceId, commCtrl, dataCtrl, port, data);
			return;
		}

		// Get configuration
		uint8_t conf = rx.buf[4] & CONF_BITS;
		// Get data from B6..B8
		data = (((uint32_t)rx.buf[5] << 16) | ((uint32_t)rx.buf[6] << 8) | (uint32_t)rx.buf[7]);

		// Validate input direction
		if (!isInput) {
			sendError(from, deviceId, commCtrl, dataCtrl, port, ERR_OPERATION_NOT_ALLOWED);
			return;
		}

		// Validate port range
		if (port == 0 || port > 16) {
			sendError(from, deviceId, commCtrl, dataCtrl, port, ERR_INVALID_PORT);
			return;
		}

		if (isWrite) {
			switch (conf) {
				case CONF_BUTTON_RISING_EDGE:
					inputConfig[port - 1].isButtonRisingEdge = data > 0;
					break;
				case CONF_BUTTON_FALLIN_EDGE:
					inputConfig[port - 1].isButtonFallingEdge = data > 0;
					break;
				case CONF_SWITCH:
					inputConfig[port - 1].isSwitch = data > 0;
					break;
				case CONF_ACTION_TOGGLE:
					inputConfig[port - 1].actionToggle = data;
					break;
				case CONF_ACTION_HIGH:
					inputConfig[port - 1].actionHigh = data;
					break;
				case CONF_ACTION_LOW:
					inputConfig[port - 1].actionLow = data;
					break;
				case CONF_DEBOUNCE:
					inputConfig[port - 1].debounce = data;
					break;
				case CONF_LONGPRESS:
					inputConfig[port - 1].longpress = data;
					break;
				case CONF_LONGPRESS_DELAYOFF:
					inputConfig[port - 1].longpressDelayOff = data;
					break;
				case CONF_BYPASS_INSTANTLY:
					inputConfig[port - 1].bypassInstantly = data > 0;
					break;
				case CONF_BYPASS_ON_DIP_SWITCH:
					inputConfig[port - 1].bypassOnDIPSwitch = data > 0;
					break;
				case CONF_BYPASS_ON_DISCONNECT:
					inputConfig[port - 1].bypassOnDisconnect = data;
					break;
				default:
					sendError(from, deviceId, commCtrl, dataCtrl, port, ERR_OPERATION_NOT_ALLOWED);
					return;
					break;	
			}
			sendAck(from, deviceId, commCtrl, dataCtrl, port, ((uint32_t)conf << 24) | data);
			return;
		} else {
			uint32_t confData = 0x0;
			switch (conf) {
				case CONF_BUTTON_RISING_EDGE:
					confData = inputConfig[port - 1].isButtonRisingEdge ? 1 : 0;
					break;
				case CONF_BUTTON_FALLIN_EDGE:
					confData = inputConfig[port - 1].isButtonFallingEdge ? 1 : 0;
					break;
				case CONF_SWITCH:
					confData = inputConfig[port - 1].isSwitch ? 1 : 0;
					break;
				case CONF_ACTION_TOGGLE:
					confData = ((uint32_t)inputConfig[port - 1].actionToggle) & 0x00FFFFFFU;
					break;
				case CONF_ACTION_HIGH:
					confData = ((uint32_t)inputConfig[port - 1].actionHigh) & 0x00FFFFFFU;
					break;
				case CONF_ACTION_LOW:
					confData = ((uint32_t)inputConfig[port - 1].actionLow) & 0x00FFFFFFU;
					break;
				case CONF_DEBOUNCE:
					confData = ((uint32_t)inputConfig[port - 1].debounce) & 0x00FFFFFFU;
					break;
				case CONF_LONGPRESS:
					confData = ((uint32_t)inputConfig[port - 1].longpress) & 0x00FFFFFFU;
					break;
				case CONF_LONGPRESS_DELAYOFF:
					confData = ((uint32_t)inputConfig[port - 1].longpressDelayOff) & 0x00FFFFFFU;
					break;
				case CONF_BYPASS_INSTANTLY:
					confData = inputConfig[port - 1].bypassInstantly ? 1 : 0;
					break;
				case CONF_BYPASS_ON_DIP_SWITCH:
					confData = inputConfig[port - 1].bypassOnDIPSwitch ? 1 : 0;
					break;
				case CONF_BYPASS_ON_DISCONNECT:
					confData = ((uint32_t)inputConfig[port - 1].bypassOnDisconnect) & 0x00FFFFFFU;
					break;
				default:
					sendError(from, deviceId, commCtrl, dataCtrl, port, ERR_OPERATION_NOT_ALLOWED);
					return;
			}
			sendAck(from, deviceId, commCtrl, dataCtrl, port, ((uint32_t)conf << 24) | confData);
			return;
		}
	}

	// Extract data from B5..B8
	data = ((uint32_t)rx.buf[4] << 24) | ((uint32_t)rx.buf[5] << 16) | ((uint32_t)rx.buf[6] << 8) | (uint32_t)rx.buf[7];

	// Command - data operation
	if (isCommand) {
		// Specific port (bit)
		if (port == 0) {
			if (isWrite) {
				// Only allow writing to output ports
				if (isInput == TYPE_INPUT) {
					sendError(from, deviceId, commCtrl, dataCtrl, port, ERR_OPERATION_NOT_ALLOWED);
					return;
				}

				// Write bitmap of values to output ports
				for (int pin = 0; pin < SIZE_OUTPUT_DIGITAL; pin++) {
					bool value = ((data >> pin) & 0x01) != 0;
					setDigitalOutput(pin, value ? HIGH : LOW, 0);
				}

				// Send back updated set of values
				sendAck(from, deviceId, commCtrl, dataCtrl, port, bitmapOutputDigitals());
			} else {
				// Send the set of digital input/output values
				sendAck(from, deviceId, commCtrl, dataCtrl, port, (isInput == TYPE_INPUT) ? bitmapInputDigitals() : bitmapOutputDigitals());
			}
		} else {
			// Single port
			if (isWrite) {
				// Only allow writing to output ports
				if (isInput == TYPE_INPUT) {
					sendError(from, deviceId, commCtrl, dataCtrl, port, ERR_OPERATION_NOT_ALLOWED);
					return;
				}

				// Write new value to output port
				if (dataType == TYPE_BIT) {
					setDigitalOutput(port-1, data & 0x01, 0);
				} else if (dataType == TYPE_INT) {
					// Set delay
					setDigitalOutput(port-1, data > 0 ? HIGH : LOW, data);
				} else {
					sendError(from, deviceId, commCtrl, dataCtrl, port, dataType);
					return;
				}

				// Send back updated set of digital input/output values
				sendAck(from, deviceId, commCtrl, dataCtrl, port, data & 0x01);
			} else {
				// Send back digital input/output value
				if (dataType == TYPE_BIT) {
					sendAck(from, deviceId, commCtrl, dataCtrl, port, (isInput == TYPE_INPUT) ? inputDigitals[port-1].value : outputDigitals[port-1].value);
				} else if (dataType == TYPE_BYTE) {
					// TODO - TYPE_BYTE
					sendError(from, deviceId, commCtrl, dataCtrl, port, ERR_UNKNOWN);
				} else if (dataType == TYPE_INT) {
					// TODO - TYPE_INT
					sendError(from, deviceId, commCtrl, dataCtrl, port, ERR_UNKNOWN);
				} else if (dataType == TYPE_FLOAT) {
					// TODO - TYPE_FLOAT
					sendError(from, deviceId, commCtrl, dataCtrl, port, ERR_UNKNOWN);
				}
			}
		}
	}
}

// Main function to setup stm32
void setup() {

	// Read DIP switches to get device id
	deviceId = computeDeviceAddress();

	// Setup digital inputs
	for (int i = 0; i < SIZE_INPUT_DIGITAL; i++) {
		// external pull-ups/downs as designed
		pinMode(inputDigitalPins[i], INPUT);
		
		// Create object
		InputDigital input{};
		input.pin         = inputDigitalPins[i];
		input.value       = digitalRead(inputDigitalPins[i]);
		input.debounce    = 0;
		input.pressedTime = 0;
		inputDigitals[i] = input;
	}

	// Setup digital outputs
	for (int i = 0; i < SIZE_OUTPUT_DIGITAL; i++) {
		// Set ouput pin mode
		pinMode(outputDigitalPins[i], OUTPUT);
		// Set default value as low
		digitalWrite(outputDigitalPins[i], LOW);

		OutputDigital output{};
		output.pin      = outputDigitalPins[i];
		output.value    = 0;
		output.delayOff = 0;
		outputDigitals[i] = output;
	}

	// Setup configuration pins
	for (int i = 0; i < 2; i++) {
		pinMode(configurationPins[i], INPUT_PULLUP);
	}

	// Read input device configuration from EEPROM
	EEPROM.get(0, inputConfig);
	// Initialize only the first time
	for (int i=0; i < SIZE_INPUT_DIGITAL; i++) {
		if (inputConfig[i].version != FIRMWARE_VERSION) {
			ConfigRegister def{};
			inputConfig[i] = def;
			resetDigitalInputConf(i);
		}
	}

	// Read configuration pins
	uint32_t canBaudRate = digitalRead(C_01) == LOW ? 1000000 : 500000;

	// Initialize CAN 
	Can1.begin();
	// Can1.setBaudRate(500000);
	Can1.setBaudRate(canBaudRate);
}

// Loop indefinetely
void loop() {

	// Calculate loop time
	loopTimeDiff = micros() - loopTimeLast;
	loopTimeLast = micros();

	// Read and process CAN messages
	CAN_message_t rx;
	while (Can1.read(rx)) {
		canProcessFrame(rx);
	}

	// Read bypass config every loop
	dipSwitchBypass = digitalRead(C_02) == LOW ? true : false;

	// Scan inputs and detect changes (for potential push events)
	for (int i = 0; i < SIZE_INPUT_DIGITAL; i++) {
		bool inputChanged = false;
		uint8_t currentValue = digitalRead(inputDigitalPins[i]) == HIGH ? 0 : 1;

		if (inputConfig[i].debounce == 0) {
			// Without debounce logic
			if (currentValue != inputDigitals[i].value) {
				inputDigitals[i].value = currentValue;
				inputChanged = true;
			}
		} else {
			// With debounce logic
			inputDigitals[i].debounce += (currentValue == HIGH ? 1 : -1) * loopTimeDiff;
			if (inputDigitals[i].debounce > inputConfig[i].debounce) {
				inputDigitals[i].debounce = inputConfig[i].debounce;
			} else if (inputDigitals[i].debounce < 0) {
				inputDigitals[i].debounce = 0;
			}

			// Debounce to logical values
			if (inputDigitals[i].debounce == inputConfig[i].debounce && inputDigitals[i].value == LOW) {
				inputDigitals[i].value = HIGH;
				inputChanged = true;
			} else if (inputDigitals[i].debounce == 0 && inputDigitals[i].value == HIGH) {
				inputDigitals[i].value = LOW;
				inputChanged = true;
			}
		}

		// Take bypass actions
		if (inputChanged) {
			if (inputConfig[i].bypassInstantly == true
				|| inputConfig[i].bypassOnDisconnect != 0 && millis() - lastSyncRemote > inputConfig[i].bypassOnDisconnect
				|| dipSwitchBypass && inputConfig[i].bypassOnDIPSwitch == true) {
				// Bypass master decisions
				if ((inputConfig[i].isButtonRisingEdge && inputDigitals[i].value == HIGH) || (inputConfig[i].isButtonFallingEdge && inputDigitals[i].value == LOW)) {
					// Toggle all matching output ports
					for (uint8_t pin = 0; pin < SIZE_OUTPUT_DIGITAL; pin++) {
						if (inputConfig[i].actionLow & (1 << pin)) {
							setDigitalOutput(pin, LOW, 0);
						}
						if (inputConfig[i].actionToggle & (1 << pin)) {
							setDigitalOutput(pin, outputDigitals[pin].value == HIGH ? LOW : HIGH, inputConfig[i].longpressDelayOff);
						}
						if (inputConfig[i].actionHigh & (1 << pin)) {
							setDigitalOutput(pin, HIGH, inputConfig[i].longpressDelayOff);
						}
					}
				} else if (inputConfig[i].isSwitch) {
					// Set all matching outputs to the input value
					for (uint8_t pin = 0; pin < SIZE_OUTPUT_DIGITAL; pin++) {
						if (inputConfig[i].actionLow & (1 << pin)) {
							setDigitalOutput(pin, LOW, 0);
						}
						if (inputConfig[i].actionToggle & (1 << pin)) {
							setDigitalOutput(pin, inputDigitals[i].value == HIGH ? HIGH : LOW, inputConfig[i].longpressDelayOff);
						}
						if (inputConfig[i].actionHigh & (1 << pin)) {
							setDigitalOutput(pin, HIGH, inputConfig[i].longpressDelayOff);
						}
					}
				}	
			}

			// Push event on input data changed
			uint8_t commCtrl = 0x00;
			uint8_t dataCtrl = DATA_DIRECTION_BIT | TYPE_BIT;
			uint32_t payload = bitmapInputDigitals();
			// Push to a broadcast address
			canWriteFrame(0xFF, deviceId, commCtrl, dataCtrl, i + 1, inputDigitals[i].value);
		}
	}

	// Watch for delay off timers on ouputs
	for (int8_t pin = 0; pin < SIZE_OUTPUT_DIGITAL; pin++) {
		if (outputDigitals[pin].delayOff > 0) {
			// Dedcut time from the output pin
			outputDigitals[pin].delayOff -= loopTimeDiff;

			// Switch off 
			if (outputDigitals[pin].delayOff <= 0) {
				setDigitalOutput(pin, LOW, 0);
			}
		}
	}
}