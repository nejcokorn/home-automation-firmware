#include <EEPROM.h>
#include "STM32_CAN.h"
#include "STM32F103Rx.h"

// Firmware
// Version is defined as 00 <Mayor> <Minor> <Bugfix>
#define FIRMWARE_VERSION  0x00000100UL

// Use for both communictaion byte and data byte
#define EMPTY_BYTE        0x00

// Communication control byte
#define COMMAND_BIT       0x80
#define DISCOVERY_BIT     0x40
#define PING_BIT          0x20
#define ACK_BIT           0x10
#define WAIT_BIT          0x08
#define ERROR_BIT         0x04

// Data control byte
#define DATA_OPERATION_BIT  0x70
#define DATA_SIGNAL_BIT     0x08
#define DATA_DIRECTION_BIT  0x04
#define DATA_TYPE_BIT       0x03

// Config control
enum class ConfigControl: uint8_t {
	configBit    = 0x80,
	operationBit = 0x40,
	optionsBit   = 0x3F,
	write        = 0x40,
	read         = 0x00,
};

// Config options
enum class ConfigOptions: uint8_t {
	writeEEPROM        = 0b00000000, // Write all configuration into EEPROM
	buttonRisingEdge   = 0b00000001, // Input acts as a Button on rising edge
	buttonFallingEdge  = 0b00000010, // Input acts as a Button on falling edge
	switcher           = 0b00000011, // Input acts as Switch
	debounce           = 0b00000100, // Debounce in microseconds
	longpress          = 0b00000101, // Longpress in milliseconds
	doubleclick        = 0b00000110, // Double-click in milliseconds
	delay              = 0b00000111, // Delay action in milliseconds
	actions            = 0b00001000, // Get/Reset all actions
	actionToggle       = 0b00001001, // Action toggle output pins
	actionHigh         = 0b00001010, // Action high output pins
	actionLow          = 0b00001011, // Action low output pins
	actionLongToggle   = 0b00001100, // Action longpress toggle output pins
	actionLongHigh     = 0b00001101, // Action longpress high output pins
	actionLongLow      = 0b00001110, // Action longpress low output pins
	actionDoubleToggle = 0b00001111, // Action double-click toggle output pins
	actionDoubleHigh   = 0b00010000, // Action double-click high output pins
	actionDoubleLow    = 0b00010001, // Action double-click low output pins
	bypassInstantly    = 0b00010010, // Bypass Instantly
	bypassOnDIPSwitch  = 0b00010011, // Bypass determined by DIP switch
	bypassOnDisconnect = 0b00010100 // Bypass on disconnect in milliseconds
};

// Protocol types
#define TYPE_DIGITAL  0
#define TYPE_ANALOG   1
#define TYPE_OUTPUT   0
#define TYPE_INPUT    1
#define TYPE_BIT      0b00 // 00 = Bit
#define TYPE_BYTE     0b01 // 01 = Byte (8-bit)
#define TYPE_INT      0b10 // 10 = Integer (32-bit)
#define TYPE_FLOAT    0b11 // 11 = Float
#define TYPE_READ     0b000 // 000 = Read
#define TYPE_WRITE    0b001 // 001 = Write
#define TYPE_TOGGLE   0b010 // 010 = Toggle
#define TYPE_RESERVED 0b11 // 11 = Reserved

// Action map types
#define TYPE_LOW            0b0000 // 0000 = LOW
#define TYPE_HIGH           0b0001 // 0001 = HIGH
#define TYPE_TOGGLE         0b0011 // 0011 = TOGGLE
#define TYPE_LONG_HIGH      0b0100 // 0100 = LOW
#define TYPE_LONG_LOW       0b0101 // 0101 = HIGH
#define TYPE_LONG_TOGGLE    0b0111 // 0111 = TOGGLE
#define TYPE_DOUBLE_HIGH    0b1000 // 1000 = LOW
#define TYPE_DOUBLE_LOW     0b1001 // 1001 = HIGH
#define TYPE_DOUBLE_TOGGLE  0b1011 // 1011 = TOGGLE

// Sizes
#define SIZE_DEVICE_ADDRESS  5
#define SIZE_INPUT_DIGITAL   16
#define SIZE_INPUT_ANALOG    4
#define SIZE_OUTPUT_DIGITAL  12
#define SIZE_ACTION_MAP      128
#define SIZE_DELAYS          128

// Error codes (packed in Data on error/ack)
#define ERR_UNKNOWN               0x00000001UL
#define ERR_OPERATION_NOT_ALLOWED 0x00000002UL
#define ERR_CONFIG_NOT_ALLOWED    0x00000003UL
#define ERR_INVALID_TYPE          0x00000004UL
#define ERR_INVALID_PORT          0x00000005UL
#define ERR_WRONG_ADDRESS         0x00000006UL

// CAN setup
STM32_CAN Can1(CAN_RX, CAN_TX, RX_SIZE_512, TX_SIZE_512);

#define CAN_BASE_ADDRESS 0x000
#define CAN_BCAST_ADDRES 0x7FF // broadcast frame receiver

uint32_t firmwareVersion;

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
	bool longpressRecorded;
};
InputDigital inputDigitals[SIZE_INPUT_DIGITAL];

struct OutputDigital {
	uint8_t pin;
	uint8_t value;
};
OutputDigital outputDigitals[SIZE_OUTPUT_DIGITAL];

struct Delay {
	bool active;
	uint8_t deviceId;
	uint8_t port;
	uint8_t type;
	int64_t time;
};
Delay delays[SIZE_DELAYS];

struct ActionMap {
	uint8_t deviceId;
	uint8_t inputPort;
	uint8_t type;
	uint16_t ports;
	uint32_t delay;
};

struct ConfigRegister {
	bool isButtonRisingEdge;
	bool isButtonFallingEdge;
	bool isSwitch;
	int32_t debounce; // trigger in microseconds
	int32_t longpress; // trigger in microseconds
	int32_t doubleclick; // trigger in microseconds
	bool bypassInstantly;
	bool bypassOnDIPSwitch;
	int32_t bypassOnDisconnect; // bypess after x miliseconds from last ping
};

// Set configuration for each input pin
ConfigRegister inputConfig[SIZE_INPUT_DIGITAL];
ActionMap actionMap[SIZE_ACTION_MAP];
ActionMap* lastActionMap = nullptr;

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
void setDigitalOutput(uint8_t port, uint8_t value) {
	if (port < SIZE_OUTPUT_DIGITAL) {
		// If value has changed, push data change frame
		if (outputDigitals[port].value != value){
			uint8_t commCtrl = 0x00;
			uint8_t dataCtrl = TYPE_BIT;
			canWriteFrame(0xFF, deviceId, commCtrl, dataCtrl, port, value);
		}

		// Do the actuall change
		digitalWrite(outputDigitalPins[port], value);
		outputDigitals[port].value = value;

		// Remove all related delays
		removeDelay(deviceId, port);
	}
}

void setDelay(uint8_t delayDeviceId, uint8_t port, uint8_t type, uint32_t delay) {
	for (uint8_t delayIdx = 0; delayIdx < SIZE_DELAYS; delayIdx++) {
		if (!delays[delayIdx].active) {
			delays[delayIdx].active = true;
			delays[delayIdx].deviceId = delayDeviceId;
			delays[delayIdx].port = port;
			delays[delayIdx].type = type;
			delays[delayIdx].time = micros() + delay * 1000;
			return;
		}
	}
}

void removeDelay(uint8_t delayDeviceId, uint8_t port) {
	for (uint8_t delayIdx = 0; delayIdx < SIZE_DELAYS; delayIdx++) {
		if (delays[delayIdx].deviceId == delayDeviceId && delays[delayIdx].port == port) {
			delays[delayIdx].active = false;
			delays[delayIdx].deviceId = 0xFF;
			delays[delayIdx].port = 0;
			delays[delayIdx].type = 0; 
			delays[delayIdx].time = 0;
			return;
		}
	}
}

void resetConfig() {
	// Reset input configuration
	for (uint8_t inputPort = 0; inputPort < SIZE_INPUT_DIGITAL; inputPort++) {
		ConfigRegister config{};
		config.isButtonRisingEdge  = false;
		config.isButtonFallingEdge = false;
		config.isSwitch            = false;
		config.debounce            = 0;
		config.longpress           = 0;
		config.doubleclick         = 0;
		config.bypassInstantly     = false;
		config.bypassOnDIPSwitch   = false;
		config.bypassOnDisconnect  = 0;
		inputConfig[inputPort] = config;
	}

	// Reset actions
	for (uint16_t i = 0; i < SIZE_ACTION_MAP; i++){
		actionMap[i].deviceId  = 0xFF;
		actionMap[i].inputPort = 0xFF;
		actionMap[i].type      = 0;
		actionMap[i].ports     = 0;
		actionMap[i].delay     = 0;
	}
}

void saveConfig() {
	uint32_t EEPROMPointer = 0;
	EEPROM.put(EEPROMPointer, FIRMWARE_VERSION);
	firmwareVersion = FIRMWARE_VERSION;
	EEPROMPointer += sizeof(FIRMWARE_VERSION);

	// Store input configuration
	for (uint8_t inputPort = 0; inputPort < SIZE_INPUT_DIGITAL; inputPort++) {
		EEPROM.put(EEPROMPointer, inputConfig[inputPort]);
		EEPROMPointer += sizeof(inputConfig[inputPort]);
	}

	// Store actions
	for (uint16_t i = 0; i < SIZE_ACTION_MAP; i++){
		EEPROM.put(EEPROMPointer, actionMap[i]);
		EEPROMPointer += sizeof(actionMap[i]);
	}
}

void readConfig() {
	uint32_t EEPROMPointer = 0;
	EEPROM.get(EEPROMPointer, firmwareVersion);
	EEPROMPointer += sizeof(firmwareVersion);

	// Read input configuration from EEPROM
	for (uint8_t inputPort = 0; inputPort < SIZE_INPUT_DIGITAL; inputPort++) {
		EEPROM.get(EEPROMPointer, inputConfig[inputPort]);
		EEPROMPointer += sizeof(inputConfig[inputPort]);
	}

	// Read actions
	for (uint16_t i = 0; i < SIZE_ACTION_MAP; i++){
		EEPROM.get(EEPROMPointer, actionMap[i]);
		EEPROMPointer += sizeof(actionMap[i]);
	}
}

void updateActionMap(uint8_t actionDeviceId, uint8_t inputPort, uint8_t type, uint16_t actionPorts) {
	// Add mapping if ports for device are defined
	for (uint16_t i = 0; i < SIZE_ACTION_MAP; i++) {
		if (actionMap[i].deviceId == 0xFF) {
			actionMap[i].deviceId = actionDeviceId;
			actionMap[i].inputPort = inputPort;
			actionMap[i].type = type;
			actionMap[i].ports = actionPorts;
			lastActionMap = &actionMap[i];
			break;
		}
	}
}

void updateActionDelay(uint32_t delay) {
	lastActionMap->delay = delay;
}

// Handle one received CAN frame for us
void canProcessFrame(const CAN_message_t& rx) {
	// Only process frames where CAN ID matches us or broadcast
	if (rx.id != (uint16_t)(CAN_BASE_ADDRESS + deviceId) && rx.id != CAN_BCAST_ADDRES) return;
	if (rx.len != 8) return;

	// Unpack payload
	uint8_t from       = rx.buf[0];        // B1
	uint8_t commCtrl   = rx.buf[1];        // B2
	uint8_t dataCtrl   = rx.buf[2];        // B3
	uint8_t configCtrl = rx.buf[2];        // B3
	uint8_t port       = rx.buf[3];        // B4
	uint32_t data = ((uint32_t)rx.buf[4] << 24) | ((uint32_t)rx.buf[5] << 16) | ((uint32_t)rx.buf[6] << 8) | (uint32_t)rx.buf[7]; // B5..B8

	// Communication control parameters
	bool isCommand     = (commCtrl & COMMAND_BIT) >> 7;
	bool isDiscovery   = (commCtrl & DISCOVERY_BIT) >> 6;
	bool isPing        = (commCtrl & PING_BIT) >> 5;
	bool isAcknowledge = (commCtrl & ACK_BIT) >> 4;
	bool isWait        = (commCtrl & WAIT_BIT) >> 3;
	bool isError       = (commCtrl & ERROR_BIT) >> 2;
	
	// Data control parameters
	uint8_t operationType = (dataCtrl & DATA_OPERATION_BIT) >> 4;
	bool isAnalog         = (dataCtrl & DATA_SIGNAL_BIT) >> 3;
	bool isInput          = (dataCtrl & DATA_DIRECTION_BIT) >> 2;
	uint8_t dataType      = (dataCtrl & DATA_TYPE_BIT);

	// Config control parameters
	bool isConfig            = (configCtrl & (uint8_t)ConfigControl::configBit) >> 7;
	bool isConfigWrite       = (configCtrl & (uint8_t)ConfigControl::operationBit) >> 6;
	uint8_t configOption     = (configCtrl & (uint8_t)ConfigControl::optionsBit);

	// Discovery
	if (isDiscovery) {
		if (isAcknowledge) {
			// Do not answer. Acknowledge sent from another device
			return;
		}
		if (rx.id != CAN_BCAST_ADDRES) {
			// Frame has to be sent to broadcast address
			return;
		}
		if (operationType != TYPE_READ || isPing || isError || isConfig) {
			sendError(from, deviceId, commCtrl, dataCtrl, port, ERR_OPERATION_NOT_ALLOWED);
			return;
		}

		// Return device firmware when asked to identify
		sendAck(from, deviceId, commCtrl | ACK_BIT, TYPE_INT << 2, 0, FIRMWARE_VERSION);
		return;
	}

	// Ping
	if (isPing) {
		if (isAcknowledge) {
			// Do not answer. Acknowledge sent from another device
			return;
		}
		if (!(rx.id == (uint16_t)(CAN_BASE_ADDRESS + deviceId) || rx.id == CAN_BCAST_ADDRES)) {
			// Frame has to be sent to broadcast address or device address
			return;
		}

		if (operationType != TYPE_READ || isError || isConfig) {
			sendError(from, deviceId, commCtrl, dataCtrl, port, ERR_OPERATION_NOT_ALLOWED);
			return;
		}

		// Pong
		sendAck(from, deviceId, commCtrl | ACK_BIT, dataCtrl, 0, 0);
		return;
	}	

	// Only reply to the broadcast address on discovery or ping frames
	if (rx.id == CAN_BCAST_ADDRES) {
		// Do not sent out error in case future devices support other functionality
		return;
	}

	// Only reply to the frames indended for this device
	if (rx.id != (uint16_t)(CAN_BASE_ADDRESS + deviceId)) {
		// Do not sent out error
		return;
	}

	// Config
	if (isConfig) {
		// Validate communication byte
		if (!isCommand || isError) {
			sendError(from, deviceId, commCtrl, configCtrl, port, ERR_OPERATION_NOT_ALLOWED);
			return;
		}

		// Write current configuration to EEPROM
		if (isConfigWrite == true && static_cast<ConfigOptions>(configOption) == ConfigOptions::writeEEPROM) {
			// Store configuration into EEPROM
			saveConfig();
			sendAck(from, deviceId, commCtrl | ACK_BIT, configCtrl, port, data);
			return;
		}

		// Validate port range
		if (port > 15) {
			sendError(from, deviceId, commCtrl, configCtrl, port, ERR_INVALID_PORT);
			return;
		}

		if (isConfigWrite == TYPE_WRITE) {
			uint8_t actionDeviceId = data >> 16;
			uint16_t actionPorts = data & 0xFFF;

			switch (static_cast<ConfigOptions>(configOption)) {
				case ConfigOptions::buttonRisingEdge:
					inputConfig[port].isButtonRisingEdge = data > 0;
					break;
				case ConfigOptions::buttonFallingEdge:
					inputConfig[port].isButtonFallingEdge = data > 0;
					break;
				case ConfigOptions::switcher:
					inputConfig[port].isSwitch = data > 0;
					break;
				case ConfigOptions::debounce:
					inputConfig[port].debounce = data;
					break;
				case ConfigOptions::longpress:
					inputConfig[port].longpress = data;
					break;
				case ConfigOptions::doubleclick:
					inputConfig[port].doubleclick = data;
					break;
				case ConfigOptions::delay:
					updateActionDelay(data);
					break;
				case ConfigOptions::actions:
					// Remove all actions for specific port
					for (uint16_t i = 0; i < SIZE_ACTION_MAP; i++) {
						if (actionMap[i].inputPort == port) {
							actionMap[i].deviceId = 0xFF;
							actionMap[i].inputPort = 0xFF;
							actionMap[i].type = 0;
							actionMap[i].ports = 0;
							actionMap[i].delay = 0;
						}
					}
					break;
				case ConfigOptions::actionToggle:
					updateActionMap(actionDeviceId, port, TYPE_TOGGLE, actionPorts);
					break;
				case ConfigOptions::actionHigh:
					updateActionMap(actionDeviceId, port, TYPE_HIGH, actionPorts);
					break;
				case ConfigOptions::actionLow:
					updateActionMap(actionDeviceId, port, TYPE_LOW, actionPorts);
					break;
				case ConfigOptions::actionLongToggle:
					updateActionMap(actionDeviceId, port, TYPE_LONG_TOGGLE, actionPorts);
					break;
				case ConfigOptions::actionLongHigh:
					updateActionMap(actionDeviceId, port, TYPE_LONG_HIGH, actionPorts);
					break;
				case ConfigOptions::actionLongLow:
					updateActionMap(actionDeviceId, port, TYPE_LONG_LOW, actionPorts);
					break;
				case ConfigOptions::actionDoubleToggle:
					updateActionMap(actionDeviceId, port, TYPE_DOUBLE_TOGGLE, actionPorts);
					break;
				case ConfigOptions::actionDoubleHigh:
					updateActionMap(actionDeviceId, port, TYPE_DOUBLE_HIGH, actionPorts);
					break;
				case ConfigOptions::actionDoubleLow:
					updateActionMap(actionDeviceId, port, TYPE_DOUBLE_LOW, actionPorts);
					break;
				case ConfigOptions::bypassInstantly:
					inputConfig[port].bypassInstantly = data > 0;
					break;
				case ConfigOptions::bypassOnDIPSwitch:
					inputConfig[port].bypassOnDIPSwitch = data > 0;
					break;
				case ConfigOptions::bypassOnDisconnect:
					inputConfig[port].bypassOnDisconnect = data;
					break;
				default:
					sendError(from, deviceId, commCtrl, configCtrl, port, ERR_OPERATION_NOT_ALLOWED);
					return;
					break;	
			}
			sendAck(from, deviceId, commCtrl | ACK_BIT, configCtrl, port, data);
			return;
		} else if (operationType == TYPE_READ) {
			uint32_t confData = 0;
			switch (static_cast<ConfigOptions>(configOption)) {
				case ConfigOptions::buttonRisingEdge:
					confData = inputConfig[port].isButtonRisingEdge ? 1 : 0;
					break;
				case ConfigOptions::buttonFallingEdge:
					confData = inputConfig[port].isButtonFallingEdge ? 1 : 0;
					break;
				case ConfigOptions::switcher:
					confData = inputConfig[port].isSwitch ? 1 : 0;
					break;
				case ConfigOptions::debounce:
					confData = ((uint32_t)inputConfig[port].debounce);
					break;
				case ConfigOptions::longpress:
					confData = ((uint32_t)inputConfig[port].longpress);
					break;
				case ConfigOptions::doubleclick:
					confData = ((uint32_t)inputConfig[port].doubleclick);
					break;
				case ConfigOptions::actions:
					ConfigOptions typeToActionConf[9];
					typeToActionConf[TYPE_LOW] = ConfigOptions::actionLow;
					typeToActionConf[TYPE_HIGH] = ConfigOptions::actionHigh;
					typeToActionConf[TYPE_TOGGLE] = ConfigOptions::actionToggle;
					typeToActionConf[TYPE_LONG_LOW] = ConfigOptions::actionLongLow;
					typeToActionConf[TYPE_LONG_HIGH] = ConfigOptions::actionLongHigh;
					typeToActionConf[TYPE_LONG_TOGGLE] = ConfigOptions::actionLongToggle;
					typeToActionConf[TYPE_DOUBLE_LOW] = ConfigOptions::actionDoubleLow;
					typeToActionConf[TYPE_DOUBLE_HIGH] = ConfigOptions::actionDoubleHigh;
					typeToActionConf[TYPE_DOUBLE_TOGGLE] = ConfigOptions::actionDoubleToggle;
					// Send configurations for output ports related to all devices on grid
					for (uint16_t i = 0; i < SIZE_ACTION_MAP; i++) {
						if (actionMap[i].inputPort == port) {
							confData = actionMap[i].deviceId << 16 | actionMap[i].ports;
							sendAck(from,
								deviceId,
								commCtrl | ACK_BIT | WAIT_BIT,
								(uint8_t)ConfigControl::configBit | (uint8_t)ConfigControl::read | (uint8_t)typeToActionConf[actionMap[i].type],
								port,
								confData);
							
							if (actionMap[i].delay != 0) {
								// Send information about action delay
								sendAck(from,
									deviceId,
									commCtrl | ACK_BIT | WAIT_BIT,
									(uint8_t)ConfigControl::configBit | (uint8_t)ConfigControl::read | (uint8_t)ConfigOptions::delay,
									port,
									actionMap[i].delay);
							}
						}
					}
					// Send last package as empty
					sendAck(from, deviceId, commCtrl | ACK_BIT, configCtrl, port, 0);
					return;
				case ConfigOptions::bypassInstantly:
					confData = inputConfig[port].bypassInstantly ? 1 : 0;
					break;
				case ConfigOptions::bypassOnDIPSwitch:
					confData = inputConfig[port].bypassOnDIPSwitch ? 1 : 0;
					break;
				case ConfigOptions::bypassOnDisconnect:
					confData = inputConfig[port].bypassOnDisconnect;
					break;
				default:
					sendError(from, deviceId, commCtrl, configCtrl, port, ERR_OPERATION_NOT_ALLOWED);
					return;
					break;
			}
			sendAck(from, deviceId, commCtrl | ACK_BIT, configCtrl, port, confData);
			return;
		}

		// Operation must be Read or Write
		sendError(from, deviceId, commCtrl, configCtrl, port, ERR_OPERATION_NOT_ALLOWED);
		return;
	}

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
				setDigitalOutput(port, data & 0x01);
			} else if (dataType == TYPE_INT) {
				// Set delay
				setDelay(deviceId, port, ((int32_t) data) > 0, data);
			} else {
				sendError(from, deviceId, commCtrl, dataCtrl, port, ERR_INVALID_TYPE);
				return;
			}

			// Send back updated value
			sendAck(from, deviceId, commCtrl | ACK_BIT, dataCtrl, port, outputDigitals[port].value);
		} else if (operationType == TYPE_TOGGLE) {
			// Only allow writing to output ports
			if (isInput == TYPE_INPUT) {
				sendError(from, deviceId, commCtrl, dataCtrl, port, ERR_OPERATION_NOT_ALLOWED);
				return;
			}

			// Toggle output value
			if (dataType == TYPE_BIT) {
				// Toggle value of output port
				setDigitalOutput(port, outputDigitals[port].value == HIGH ? LOW : HIGH);
			} else if (dataType == TYPE_INT) {
				// Set delay
				setDelay(deviceId, port, outputDigitals[port].value  == HIGH ? LOW : HIGH, data);
			} else {
				sendError(from, deviceId, commCtrl, dataCtrl, port, ERR_INVALID_TYPE);
				return;
			}

			// Send back updated value
			sendAck(from, deviceId, commCtrl | ACK_BIT, dataCtrl, port, outputDigitals[port].value);
		} else if (operationType == TYPE_READ) {
			// Send back digital input/output value
			if (dataType == TYPE_BIT) {
				sendAck(from, deviceId, commCtrl | ACK_BIT, dataCtrl, port, (isInput == TYPE_INPUT) ? inputDigitals[port].value : outputDigitals[port].value);
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

	// No action could be taken based on the frame definition
	sendError(from, deviceId, commCtrl, dataCtrl, port, ERR_OPERATION_NOT_ALLOWED);
	return;
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
		input.pin               = inputDigitalPins[inputPort];
		input.value             = digitalRead(inputDigitalPins[inputPort]);
		input.debounce          = 0;
		input.pressedTime       = 0;
		input.longpressRecorded = 0;
		inputDigitals[inputPort] = input;
	}

	// Setup digital outputs
	for (int outputPort = 0; outputPort < SIZE_OUTPUT_DIGITAL; outputPort++) {
		// Set ouput pin mode
		pinMode(outputDigitalPins[outputPort], OUTPUT);
		// Set default value as low
		digitalWrite(outputDigitalPins[outputPort], LOW);

		OutputDigital output{};
		output.pin        = outputDigitalPins[outputPort];
		output.value      = 0;
		outputDigitals[outputPort] = output;
	}

	// Setup dalays
	for (uint8_t delayIdx = 0; delayIdx < SIZE_DELAYS; delayIdx++) {
		Delay delay{};
		delay.active   = false;
		delay.deviceId = 0xFF;
		delay.port     = 0;
		delay.type     = 0;
		delay.time     = 0;
		delays[delayIdx] = delay;
	}

	// Setup configuration pins
	for (int pin = 0; pin < 2; pin++) {
		pinMode(configurationPins[pin], INPUT_PULLUP);
	}
	
	// Read configuration from EEPROM
	readConfig();

	// Compare version value against value in EEPROM
	if (firmwareVersion != FIRMWARE_VERSION) {
		// Reset input configuration and action map configurations
		resetConfig();

		// Store defaults into EEPROM
		saveConfig();
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

		if ((inputConfig[inputPort].isButtonRisingEdge && inputDigitals[inputPort].value == HIGH) || (inputConfig[inputPort].isButtonFallingEdge && inputDigitals[inputPort].value == LOW)) {
			inputDigitals[inputPort].pressedTime += loopTimeDiff;
		} else {
			inputDigitals[inputPort].longpressRecorded = false;
			inputDigitals[inputPort].pressedTime = 0;
		}

		// Take bypass actions
		if (inputChanged) {
			// Push event on input data changed
			uint8_t commCtrl = EMPTY_BYTE;
			uint8_t dataCtrl = DATA_DIRECTION_BIT | TYPE_BIT;
			// Push to a broadcast address
			canWriteFrame(0xFF, deviceId, commCtrl, dataCtrl, inputPort, inputDigitals[inputPort].value);

			// Calculate bypass
			if (inputConfig[inputPort].bypassInstantly == true
				|| inputConfig[inputPort].bypassOnDisconnect != 0 && millis() - lastSyncRemote > inputConfig[inputPort].bypassOnDisconnect
				|| dipSwitchBypass && inputConfig[inputPort].bypassOnDIPSwitch == true) {
				// Bypass master decisions
				if (inputConfig[inputPort].isSwitch || (inputConfig[inputPort].isButtonRisingEdge && inputDigitals[inputPort].value == HIGH) || (inputConfig[inputPort].isButtonFallingEdge && inputDigitals[inputPort].value == LOW)) {
					// Loop actions
					for (uint16_t gridDevIdx = 0; gridDevIdx < SIZE_ACTION_MAP; gridDevIdx++) {
						if (actionMap[gridDevIdx].deviceId != 0xFF && actionMap[gridDevIdx].inputPort == inputPort) {
							for (uint8_t outputPort = 0; outputPort < SIZE_OUTPUT_DIGITAL; outputPort++) {
								if (actionMap[gridDevIdx].ports & (1 << outputPort)) {
									if (actionMap[gridDevIdx].delay > 0) {	
										// Change value of local output port
										if (actionMap[gridDevIdx].type == TYPE_HIGH) {
											setDelay(actionMap[gridDevIdx].deviceId, outputPort, TYPE_HIGH, actionMap[gridDevIdx].delay);
										} else if (actionMap[gridDevIdx].type == TYPE_LOW) {
											setDelay(actionMap[gridDevIdx].deviceId, outputPort, TYPE_LOW, actionMap[gridDevIdx].delay);
										} else if (actionMap[gridDevIdx].type == TYPE_TOGGLE) {
											setDelay(actionMap[gridDevIdx].deviceId, outputPort, TYPE_TOGGLE, actionMap[gridDevIdx].delay);
										}
									} else if (actionMap[gridDevIdx].deviceId == deviceId) {
										// Change value of local output port
										if (actionMap[gridDevIdx].type == TYPE_HIGH) {
											setDigitalOutput(outputPort, HIGH);
										} else if (actionMap[gridDevIdx].type == TYPE_LOW) {
											setDigitalOutput(outputPort, LOW);
										} else if (actionMap[gridDevIdx].type == TYPE_TOGGLE) {
											uint8_t desiredState = outputDigitals[outputPort].value == HIGH ? LOW : HIGH;
											setDigitalOutput(outputPort, desiredState);
										}
									} else {
										// Send command to change remote output port
										if (actionMap[gridDevIdx].type == TYPE_HIGH) {
											canWriteFrame(actionMap[gridDevIdx].deviceId, deviceId, COMMAND_BIT, TYPE_WRITE << 4, outputPort, HIGH);
										} else if (actionMap[gridDevIdx].type == TYPE_LOW) {
											canWriteFrame(actionMap[gridDevIdx].deviceId, deviceId, COMMAND_BIT, TYPE_WRITE << 4, outputPort, LOW);
										} else if (actionMap[gridDevIdx].type == TYPE_TOGGLE) {
											canWriteFrame(actionMap[gridDevIdx].deviceId, deviceId, COMMAND_BIT, TYPE_TOGGLE << 4, outputPort, 0);
										}
									}
								}
							}
						}
					}
				}
			}
		}

		// Bypass actions on longpress
		if (inputDigitals[inputPort].longpressRecorded == false && inputDigitals[inputPort].pressedTime > inputConfig[inputPort].longpress * 1000) {
			inputDigitals[inputPort].longpressRecorded = true;
			// Push event on input longpress
			uint8_t commCtrl = EMPTY_BYTE;
			uint8_t dataCtrl = DATA_DIRECTION_BIT | TYPE_BIT;
			canWriteFrame(0xFF, deviceId, commCtrl, dataCtrl, inputPort, inputDigitals[inputPort].pressedTime / 1000);


			// Calculate bypass
			if (inputConfig[inputPort].bypassInstantly == true
				|| inputConfig[inputPort].bypassOnDisconnect != 0 && millis() - lastSyncRemote > inputConfig[inputPort].bypassOnDisconnect
				|| dipSwitchBypass && inputConfig[inputPort].bypassOnDIPSwitch == true) {

				// Action longpress events
				for (uint16_t gridDevIdx = 0; gridDevIdx < SIZE_ACTION_MAP; gridDevIdx++) {
					if (actionMap[gridDevIdx].deviceId != 0xFF && actionMap[gridDevIdx].inputPort == inputPort) {
						if (actionMap[gridDevIdx].type == TYPE_HIGH) {
							for (uint8_t outputPort = 0; outputPort < SIZE_OUTPUT_DIGITAL; outputPort++) {
								if (actionMap[gridDevIdx].ports & (1 << outputPort)) {
									if (actionMap[gridDevIdx].deviceId == deviceId) {
										// Change value of local output port
										if (actionMap[gridDevIdx].type == TYPE_LONG_HIGH) {
											setDigitalOutput(outputPort, HIGH);
										} else if (actionMap[gridDevIdx].type == TYPE_LONG_LOW) {
											setDigitalOutput(outputPort, LOW);
										} else if (actionMap[gridDevIdx].type == TYPE_LONG_TOGGLE) {
											setDigitalOutput(outputPort, outputDigitals[outputPort].value == HIGH ? LOW : HIGH);
										}
									} else {
										// Send command to change output port on deviceId
										if (actionMap[gridDevIdx].type == TYPE_LONG_HIGH) {
											canWriteFrame(actionMap[gridDevIdx].deviceId, deviceId, COMMAND_BIT, TYPE_WRITE << 4, outputPort, HIGH);
										} else if (actionMap[gridDevIdx].type == TYPE_LONG_LOW) {
											canWriteFrame(actionMap[gridDevIdx].deviceId, deviceId, COMMAND_BIT, TYPE_WRITE << 4, outputPort, 0);
										} else if (actionMap[gridDevIdx].type == TYPE_LONG_TOGGLE) {
											canWriteFrame(actionMap[gridDevIdx].deviceId, deviceId, COMMAND_BIT, TYPE_TOGGLE << 4, outputPort, 0);
										}
									}
								}
							}
						}
					}
				}
			}
		}
	}

	// Watch for delay timers
	for (uint8_t delayIdx = 0; delayIdx < SIZE_DELAYS; delayIdx++) {
		if (delays[delayIdx].active == true && delays[delayIdx].time < micros()) {
			if (delays[delayIdx].deviceId == deviceId) {
				if (delays[delayIdx].type == TYPE_TOGGLE) {
					uint8_t desiredState = outputDigitals[delays[delayIdx].port].value == HIGH ? LOW : HIGH;
					setDigitalOutput(delays[delayIdx].port, desiredState);
				} else {
					setDigitalOutput(delays[delayIdx].port, delays[delayIdx].type);
				}
			} else {
				if (delays[delayIdx].type == TYPE_TOGGLE) {
					canWriteFrame(delays[delayIdx].deviceId, deviceId, COMMAND_BIT, TYPE_TOGGLE << 4, delays[delayIdx].port, 0);
				} else {
					canWriteFrame(delays[delayIdx].deviceId, deviceId, COMMAND_BIT, TYPE_WRITE << 4, delays[delayIdx].port, delays[delayIdx].type);
				}
			}
			removeDelay(delays[delayIdx].deviceId, delays[delayIdx].port);
		}
	}
}