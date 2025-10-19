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
#define DATA_OPERATION_BIT  0x30
#define DATA_SIGNAL_BIT     0x08
#define DATA_DIRECTION_BIT  0x04
#define DATA_TYPE_BIT       0x03

// ConfCtrl Byte Enums
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
#define TYPE_DIGITAL  0
#define TYPE_ANALOG   1
#define TYPE_OUTPUT   0
#define TYPE_INPUT    1
#define TYPE_BIT      0b00 // 00 = Bit
#define TYPE_BYTE     0b01 // 01 = Byte (8-bit)
#define TYPE_INT      0b10 // 10 = Integer (32-bit)
#define TYPE_FLOAT    0b11 // 11 = Float
#define TYPE_READ     0b00 // 00 = Read
#define TYPE_WRITE    0b01 // 01 = Write
#define TYPE_TOGGLE   0b10 // 10 = Toggle
#define TYPE_RESERVED 0b11 // 11 = Reserved

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
	for (int pin = 0; pin < SIZE_DEVICE_ADDRESS; pin++) {
		pinMode(deviceAddressPins[pin], INPUT_PULLUP);
		// LOW means switch ON -> bit=1
		id |= ((digitalRead(deviceAddressPins[pin]) == LOW) ? 1 : 0) << pin;
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
	bool isConfig         = (dataCtrl & DATA_CONFIG_BIT) >> 7;
	bool isWriteEEPROM    = (dataCtrl & DATA_EEPROM_BIT) >> 6;
	uint8_t operationType = (dataCtrl & DATA_OPERATION_BIT) >> 4;
	bool isAnalog         = (dataCtrl & DATA_SIGNAL_BIT) >> 3;
	bool isInput          = (dataCtrl & DATA_DIRECTION_BIT) >> 2;
	uint8_t dataType      = (dataCtrl & DATA_TYPE_BIT);

	// Discovery
	// Return device address when asked to identify
	if (rx.id == CAN_BCAST_ADDRES && isDiscovery && operationType == TYPE_READ && !isConfig) {
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
		uint8_t conf = rx.buf[4];
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

		if (operationType == TYPE_WRITE) {
			switch (conf) {
				case CONF_BUTTON_RISING_EDGE:
					inputConfig[port].isButtonRisingEdge = data > 0;
					break;
				case CONF_BUTTON_FALLIN_EDGE:
					inputConfig[port].isButtonFallingEdge = data > 0;
					break;
				case CONF_SWITCH:
					inputConfig[port].isSwitch = data > 0;
					break;
				case CONF_ACTION_TOGGLE:
					inputConfig[port].actionToggle = data;
					break;
				case CONF_ACTION_HIGH:
					inputConfig[port].actionHigh = data;
					break;
				case CONF_ACTION_LOW:
					inputConfig[port].actionLow = data;
					break;
				case CONF_DEBOUNCE:
					inputConfig[port].debounce = data;
					break;
				case CONF_LONGPRESS:
					inputConfig[port].longpress = data;
					break;
				case CONF_LONGPRESS_DELAYOFF:
					inputConfig[port].longpressDelayOff = data;
					break;
				case CONF_BYPASS_INSTANTLY:
					inputConfig[port].bypassInstantly = data > 0;
					break;
				case CONF_BYPASS_ON_DIP_SWITCH:
					inputConfig[port].bypassOnDIPSwitch = data > 0;
					break;
				case CONF_BYPASS_ON_DISCONNECT:
					inputConfig[port].bypassOnDisconnect = data;
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
					confData = inputConfig[port].isButtonRisingEdge ? 1 : 0;
					break;
				case CONF_BUTTON_FALLIN_EDGE:
					confData = inputConfig[port].isButtonFallingEdge ? 1 : 0;
					break;
				case CONF_SWITCH:
					confData = inputConfig[port].isSwitch ? 1 : 0;
					break;
				case CONF_ACTION_TOGGLE:
					confData = ((uint32_t)inputConfig[port].actionToggle) & 0x00FFFFFFU;
					break;
				case CONF_ACTION_HIGH:
					confData = ((uint32_t)inputConfig[port].actionHigh) & 0x00FFFFFFU;
					break;
				case CONF_ACTION_LOW:
					confData = ((uint32_t)inputConfig[port].actionLow) & 0x00FFFFFFU;
					break;
				case CONF_DEBOUNCE:
					confData = ((uint32_t)inputConfig[port].debounce) & 0x00FFFFFFU;
					break;
				case CONF_LONGPRESS:
					confData = ((uint32_t)inputConfig[port].longpress) & 0x00FFFFFFU;
					break;
				case CONF_LONGPRESS_DELAYOFF:
					confData = ((uint32_t)inputConfig[port].longpressDelayOff) & 0x00FFFFFFU;
					break;
				case CONF_BYPASS_INSTANTLY:
					confData = inputConfig[port].bypassInstantly ? 1 : 0;
					break;
				case CONF_BYPASS_ON_DIP_SWITCH:
					confData = inputConfig[port].bypassOnDIPSwitch ? 1 : 0;
					break;
				case CONF_BYPASS_ON_DISCONNECT:
					confData = ((uint32_t)inputConfig[port].bypassOnDisconnect) & 0x00FFFFFFU;
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
		if (operationType == TYPE_WRITE) {
			// Only allow writing to output ports
			if (isInput == TYPE_INPUT) {
				sendError(from, deviceId, commCtrl, dataCtrl, port, ERR_OPERATION_NOT_ALLOWED);
				return;
			}

			// Set output value
			if (dataType == TYPE_BIT) {
				// Write new value to output port
				setDigitalOutput(port, data & 0x01, 0);
			} else if (dataType == TYPE_INT) {
				// Set delayOff
				setDigitalOutput(port, data > 0 ? HIGH : LOW, data);
			} else {
				sendError(from, deviceId, commCtrl, dataCtrl, port, ERR_INVALID_TYPE);
				return;
			}

			// Send back updated value
			sendAck(from, deviceId, commCtrl, dataCtrl, port, outputDigitals[port].value + outputDigitals[port].delayOff);
		} else if (operationType == TYPE_TOGGLE) {
			// Only allow writing to output ports
			if (isInput == TYPE_INPUT) {
				sendError(from, deviceId, commCtrl, dataCtrl, port, ERR_OPERATION_NOT_ALLOWED);
				return;
			}

			// Toggle output value
			if (dataType == TYPE_BIT) {
				// Toggle value of output port
				setDigitalOutput(port, outputDigitals[port].value == HIGH ? LOW : HIGH, 0);
			} else if (dataType == TYPE_INT) {
				// Set delayOff
				setDigitalOutput(port, outputDigitals[port].value  == HIGH ? LOW : HIGH, data);
			} else {
				sendError(from, deviceId, commCtrl, dataCtrl, port, ERR_INVALID_TYPE);
				return;
			}

			// Send back updated value
			sendAck(from, deviceId, commCtrl, dataCtrl, port, outputDigitals[port].value);
		} else if (operationType == TYPE_READ) {
			// Send back digital input/output value
			if (dataType == TYPE_BIT) {
				sendAck(from, deviceId, commCtrl, dataCtrl, port, (isInput == TYPE_INPUT) ? inputDigitals[port].value : outputDigitals[port].value);
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

// Main function to setup stm32
void setup() {

	// Read DIP switches to get device id
	deviceId = computeDeviceAddress();

	// Setup digital inputs
	for (int inputPort = 0; inputPort < SIZE_INPUT_DIGITAL; inputPort++) {
		// external pull-ups/downs as designed
		pinMode(inputDigitalPins[inputPort], INPUT);
		
		// Create object
		InputDigital input{};
		input.pin         = inputDigitalPins[inputPort];
		input.value       = digitalRead(inputDigitalPins[inputPort]);
		input.debounce    = 0;
		input.pressedTime = 0;
		inputDigitals[inputPort] = input;
	}

	// Setup digital outputs
	for (int outputPort = 0; outputPort < SIZE_OUTPUT_DIGITAL; outputPort++) {
		// Set ouput pin mode
		pinMode(outputDigitalPins[outputPort], OUTPUT);
		// Set default value as low
		digitalWrite(outputDigitalPins[outputPort], LOW);

		OutputDigital output{};
		output.pin      = outputDigitalPins[outputPort];
		output.value    = 0;
		output.delayOff = 0;
		outputDigitals[outputPort] = output;
	}

	// Setup configuration pins
	for (int pin = 0; pin < 2; pin++) {
		pinMode(configurationPins[pin], INPUT_PULLUP);
	}

	// Read input device configuration from EEPROM
	EEPROM.get(0, inputConfig);
	// Initialize only the first time
	for (int inputPort=0; inputPort < SIZE_INPUT_DIGITAL; inputPort++) {
		if (inputConfig[inputPort].version != FIRMWARE_VERSION) {
			ConfigRegister def{};
			inputConfig[inputPort] = def;
			resetDigitalInputConf(inputPort);
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
	for (int inputPort = 0; inputPort < SIZE_INPUT_DIGITAL; inputPort++) {
		bool inputChanged = false;
		uint8_t currentValue = digitalRead(inputDigitalPins[inputPort]) == HIGH ? 0 : 1;

		if (inputConfig[inputPort].debounce == 0) {
			// Without debounce logic
			if (currentValue != inputDigitals[inputPort].value) {
				inputDigitals[inputPort].value = currentValue;
				inputChanged = true;
			}
		} else {
			// With debounce logic
			inputDigitals[inputPort].debounce += (currentValue == HIGH ? 1 : -1) * loopTimeDiff;
			if (inputDigitals[inputPort].debounce > inputConfig[inputPort].debounce) {
				inputDigitals[inputPort].debounce = inputConfig[inputPort].debounce;
			} else if (inputDigitals[inputPort].debounce < 0) {
				inputDigitals[inputPort].debounce = 0;
			}

			// Debounce to logical values
			if (inputDigitals[inputPort].debounce == inputConfig[inputPort].debounce && inputDigitals[inputPort].value == LOW) {
				inputDigitals[inputPort].value = HIGH;
				inputChanged = true;
			} else if (inputDigitals[inputPort].debounce == 0 && inputDigitals[inputPort].value == HIGH) {
				inputDigitals[inputPort].value = LOW;
				inputChanged = true;
			}
		}

		// Take bypass actions
		if (inputChanged) {
			if (inputConfig[inputPort].bypassInstantly == true
				|| inputConfig[inputPort].bypassOnDisconnect != 0 && millis() - lastSyncRemote > inputConfig[inputPort].bypassOnDisconnect
				|| dipSwitchBypass && inputConfig[inputPort].bypassOnDIPSwitch == true) {
				// Bypass master decisions
				if ((inputConfig[inputPort].isButtonRisingEdge && inputDigitals[inputPort].value == HIGH) || (inputConfig[inputPort].isButtonFallingEdge && inputDigitals[inputPort].value == LOW)) {
					// Toggle all matching output ports
					for (uint8_t outputPort = 0; outputPort < SIZE_OUTPUT_DIGITAL; outputPort++) {
						if (inputConfig[inputPort].actionLow & (1 << outputPort)) {
							setDigitalOutput(outputPort, LOW, 0);
						}
						if (inputConfig[inputPort].actionToggle & (1 << outputPort)) {
							setDigitalOutput(outputPort, outputDigitals[outputPort].value == HIGH ? LOW : HIGH, inputConfig[inputPort].longpressDelayOff);
						}
						if (inputConfig[inputPort].actionHigh & (1 << outputPort)) {
							setDigitalOutput(outputPort, HIGH, inputConfig[inputPort].longpressDelayOff);
						}
					}
				} else if (inputConfig[inputPort].isSwitch) {
					// Set all matching outputs to the input value
					for (uint8_t outputPort = 0; outputPort < SIZE_OUTPUT_DIGITAL; outputPort++) {
						if (inputConfig[inputPort].actionLow & (1 << outputPort)) {
							setDigitalOutput(outputPort, LOW, 0);
						}
						if (inputConfig[inputPort].actionToggle & (1 << outputPort)) {
							setDigitalOutput(outputPort, inputDigitals[inputPort].value == HIGH ? HIGH : LOW, inputConfig[inputPort].longpressDelayOff);
						}
						if (inputConfig[inputPort].actionHigh & (1 << outputPort)) {
							setDigitalOutput(outputPort, HIGH, inputConfig[inputPort].longpressDelayOff);
						}
					}
				}	
			}

			// Push event on input data changed
			uint8_t commCtrl = 0x00;
			uint8_t dataCtrl = DATA_DIRECTION_BIT | TYPE_BIT;
			// Push to a broadcast address
			canWriteFrame(0xFF, deviceId, commCtrl, dataCtrl, inputPort, inputDigitals[inputPort].value);
		}
	}

	// Watch for delay off timers on ouputs
	for (int8_t outputPort = 0; outputPort < SIZE_OUTPUT_DIGITAL; outputPort++) {
		if (outputDigitals[outputPort].delayOff > 0) {
			// Dedcut time from the output port
			outputDigitals[outputPort].delayOff -= loopTimeDiff;

			// Switch off 
			if (outputDigitals[outputPort].delayOff <= 0) {
				setDigitalOutput(outputPort, LOW, 0);
			}
		}
	}
}