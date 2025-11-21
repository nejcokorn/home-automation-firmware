#include <EEPROM.h>
#include "STM32_CAN.h"
#include "STM32F103Rx.h"

// Firmware
// Version is defined as 00 <Mayor> <Minor> <Bugfix>
#define FIRMWARE_VERSION  0x00000100UL

// Communication control byte
enum class CommunicationCtrl: uint8_t {
	empty          = 0x00,
	commandBit     = 0x80,
	discoveryBit   = 0x40,
	pingBit        = 0x20,
	acknowledgeBit = 0x10,
	waitBit        = 0x08,
	errorBit       = 0x04,
};

// Data control
enum class DataCtrl: uint8_t {
	empty         = 0x00,
	operationBits = 0x70,
	signalBit     = 0x08,
	directionBit  = 0x04,
	dataTypeBits  = 0x03,
	// Specific Types
	get           = 0x00,
	set           = 0x10,
	extra         = 0x20,
	delay         = 0x30,
	listDelays    = 0x40,
	digital       = 0x00,
	analog        = 0x08,
	input         = 0x04,
	output        = 0x00,
	bit           = 0x00,
	byte          = 0x01,
	integer       = 0x02,
	decimal       = 0x03,
};

// Config control
enum class ConfigCtrl: uint8_t {
	empty        = 0x00,
	configBit    = 0x80,
	operationBit = 0x40,
	optionBits   = 0x3F,
	// Specific Types
	get          = 0x00,
	set          = 0x40,
};

// Config options
enum class ConfigOptions: uint8_t {
	writeEEPROM             = 0b00000000, // Write all configuration into EEPROM
	buttonRisingEdge        = 0b00000001, // Input acts as a Button on rising edge
	buttonFallingEdge       = 0b00000010, // Input acts as a Button on falling edge
	switcher                = 0b00000011, // Input acts as Switch
	debounce                = 0b00000100, // Debounce in microseconds
	longpress               = 0b00000101, // Longpress in milliseconds
	doubleclick             = 0b00000110, // Double-click in milliseconds
	delay                   = 0b00000111, // Delay action in milliseconds
	actions                 = 0b00001000, // Get/Reset all actions
	actionToggle            = 0b00001001, // Action toggle output pins
	actionHigh              = 0b00001010, // Action high output pins
	actionLow               = 0b00001011, // Action low output pins
	actionLongpressToggle   = 0b00001100, // Action longpress toggle output pins
	actionLongpressHigh     = 0b00001101, // Action longpress high output pins
	actionLongpressLow      = 0b00001110, // Action longpress low output pins
	actionDoubleclickToggle = 0b00001111, // Action double-click toggle output pins
	actionDoubleclickHigh   = 0b00010000, // Action double-click high output pins
	actionDoubleclickLow    = 0b00010001, // Action double-click low output pins
	bypassInstantly         = 0b00010010, // Bypass Instantly
	bypassOnDIPSwitch       = 0b00010011, // Bypass determined by DIP switch
	bypassOnDisconnect      = 0b00010100, // Bypass on disconnect in milliseconds
};

// Action types
enum class ActionType: uint8_t {
	low    = LOW,
	high   = HIGH,
	toggle = 0b10,
	pwm    = 0b11,
};

enum class ActionMode: uint8_t {
	normal      = 0b00,
	longpress   = 0b01,
	doubleclick = 0b10,
};

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

#define CAN_BCAST_ADDRES 0xFF // broadcast frame receiver

uint32_t firmwareVersion;

// Logical device identity
uint8_t thisDeviceId = 0;

// Hardware pin maps (from your original sketch)
const int deviceAddressPins[] = { DEV_A1, DEV_A2, DEV_A3, DEV_A4, DEV_A5 };
const int inputDigitalPins[]  = { DI_01, DI_02, DI_03, DI_04, DI_05, DI_06, DI_07, DI_08, DI_09, DI_10, DI_11, DI_12, DI_13, DI_14, DI_15, DI_16 };
const int outputDigitalPins[] = { DO_01, DO_02, DO_03, DO_04, DO_05, DO_06, DO_07, DO_08, DO_09, DO_10, DO_11, DO_12 };
const int configurationPins[] = { C_01, C_02 };


struct InputDigital {
	uint8_t pin;
	uint8_t value;
	int32_t debounce;
	uint64_t clickTime;
	uint64_t previousClickTime;
	bool processClick;
	bool processDoubleclick;
	bool processLongpress;
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
	ActionType type;
	int64_t time;
};
Delay delays[SIZE_DELAYS];

struct ActionItem {
	uint8_t deviceId;
	uint8_t inputPort;
	ActionType type;
	ActionMode mode;
	uint16_t ports;
	uint32_t delay;
};

struct Command {
	bool isInput;
	bool isOutput;
	bool isGet;
	bool isSet;
	bool isAnalog;
	bool isDigital;
	bool isBit;
	bool isByte;
	bool isInteger;
	bool isDecimal;
	uint8_t port;
	uint32_t delay;
	uint32_t extra;
	ActionType type;
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
ActionItem actionItems[SIZE_ACTION_MAP];
ActionItem* lastActionItem = nullptr;
Command execCommand;
ConfigOptions actionToConfigType[3][3];

// Global variables
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
	tx.id  = to;
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
	// Set acknowledgeBit, ensure no errorBit
	canWriteFrame(
		to,
		from,
		(commCtrl | (uint8_t)CommunicationCtrl::acknowledgeBit) & ~(uint8_t)CommunicationCtrl::errorBit,
		dataCtrl,
		port,
		data);
}

// Send error frame
void sendError(uint8_t to, uint8_t from, uint8_t commCtrl, uint8_t dataCtrl, uint8_t port, uint32_t errCode) {
	// Set acknowledgeBit and errorBit
	canWriteFrame(
		to,
		from,
		(commCtrl | (uint8_t)CommunicationCtrl::acknowledgeBit) & (uint8_t)CommunicationCtrl::errorBit,
		dataCtrl,
		port,
		errCode);
}

// Change status of the output port
void setDigitalOutput(uint8_t port, ActionType actionType) {
	if (port < SIZE_OUTPUT_DIGITAL) {
		// Calculate new value
		uint32_t value;
		if (actionType == ActionType::toggle){
			value = outputDigitals[port].value == HIGH ? LOW : HIGH;
		} else {
			// LOW/HIGH
			value = (uint32_t)actionType;
		}
		// If value has changed, push data change frame
		if (outputDigitals[port].value != value){
			uint8_t commCtrl = 0x00;
			uint8_t dataCtrl = (uint8_t)DataCtrl::bit;
			canWriteFrame(0xFF, thisDeviceId, commCtrl, dataCtrl, port, value);
		}

		// Do the actuall change
		digitalWrite(outputDigitalPins[port], value);
		outputDigitals[port].value = value;

		// Remove all related delays
		removeDelay(thisDeviceId, port);
	}
}

void setDelay(uint8_t deviceId, uint8_t port, ActionType type, uint32_t delay) {
	for (uint8_t delayIdx = 0; delayIdx < SIZE_DELAYS; delayIdx++) {
		if (!delays[delayIdx].active) {
			delays[delayIdx].active = true;
			delays[delayIdx].deviceId = deviceId;
			delays[delayIdx].port = port;
			delays[delayIdx].type = type;
			delays[delayIdx].time = micros() + delay * 1000;
			return;
		}
	}
}

void removeDelay(uint8_t deviceId, uint8_t port) {
	for (uint8_t delayIdx = 0; delayIdx < SIZE_DELAYS; delayIdx++) {
		if (delays[delayIdx].deviceId == deviceId && delays[delayIdx].port == port) {
			delays[delayIdx].active = false;
			delays[delayIdx].deviceId = 0xFF;
			delays[delayIdx].port = 0;
			delays[delayIdx].type = ActionType::low; 
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
		actionItems[i].deviceId  = 0xFF;
		actionItems[i].inputPort = 0xFF;
		actionItems[i].mode      = ActionMode::normal;
		actionItems[i].type      = ActionType::low;
		actionItems[i].ports     = 0;
		actionItems[i].delay     = 0;
	}
}

uint32_t saveConfig() {
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
		EEPROM.put(EEPROMPointer, actionItems[i]);
		EEPROMPointer += sizeof(actionItems[i]);
	}
	return EEPROMPointer;
}

void readConfig() {
	uint32_t EEPROMPointer = 0;
	EEPROM.get(EEPROMPointer, firmwareVersion);
	EEPROMPointer += sizeof(firmwareVersion);

	// Get input configuration from EEPROM
	for (uint8_t inputPort = 0; inputPort < SIZE_INPUT_DIGITAL; inputPort++) {
		EEPROM.get(EEPROMPointer, inputConfig[inputPort]);
		EEPROMPointer += sizeof(inputConfig[inputPort]);
	}

	// Get actions from EEPROM
	for (uint16_t i = 0; i < SIZE_ACTION_MAP; i++){
		EEPROM.get(EEPROMPointer, actionItems[i]);
		EEPROMPointer += sizeof(actionItems[i]);
	}
}

void updateActionItem(uint8_t deviceId, uint8_t inputPort, ActionMode mode, ActionType type, uint16_t actionPorts) {
	// Add mapping if ports for device are defined
	for (uint16_t i = 0; i < SIZE_ACTION_MAP; i++) {
		if (actionItems[i].deviceId == 0xFF) {
			actionItems[i].deviceId = deviceId;
			actionItems[i].inputPort = inputPort;
			actionItems[i].mode = mode;
			actionItems[i].type = type;
			actionItems[i].ports = actionPorts;
			lastActionItem = &actionItems[i];
			break;
		}
	}
}

void updateActionDelay(uint32_t delay) {
	lastActionItem->delay = delay;
}

void resetCommand(Command* execCommand) {
	execCommand->isSet     = false;
	execCommand->isDigital = false;
	execCommand->isAnalog  = false;
	execCommand->isOutput  = false;
	execCommand->isBit     = false;
	execCommand->isByte    = false;
	execCommand->isInteger = false;
	execCommand->isDecimal = false;
	execCommand->port      = 0xFF;
	execCommand->type      = ActionType::low;
	execCommand->delay     = 0;
	execCommand->extra     = 0;
}

void execBypass(uint8_t gridDevIdx) {
	for (uint8_t outputPort = 0; outputPort < SIZE_OUTPUT_DIGITAL; outputPort++) {
		if (actionItems[gridDevIdx].ports & (1 << outputPort)) {
			if (actionItems[gridDevIdx].delay > 0) {	
				// Change value of local output port
				setDelay(actionItems[gridDevIdx].deviceId, outputPort, actionItems[gridDevIdx].type, actionItems[gridDevIdx].delay);
			} else if (actionItems[gridDevIdx].deviceId == thisDeviceId) {
				// Change value of local output port
				setDigitalOutput(outputPort, actionItems[gridDevIdx].type);
			} else {
				// Send command to change remote output port
				canWriteFrame(actionItems[gridDevIdx].deviceId, thisDeviceId, (uint8_t)CommunicationCtrl::commandBit, (uint8_t)DataCtrl::set, outputPort, (uint8_t)actionItems[gridDevIdx].type);
			}
		}
	}
}

// Handle one received CAN frame for us
void canProcessFrame(const CAN_message_t& rx) {
	// Only process frames where CAN messages where length is 8 bytes
	if (rx.len != 8) return;

	// Unpack payload
	uint8_t from       = rx.buf[0];        // B1
	uint8_t commCtrl   = rx.buf[1];        // B2
	uint8_t dataCtrl   = rx.buf[2];        // B3
	uint8_t configCtrl = rx.buf[2];        // B3
	uint8_t port       = rx.buf[3];        // B4
	uint32_t data = ((uint32_t)rx.buf[4] << 24) | ((uint32_t)rx.buf[5] << 16) | ((uint32_t)rx.buf[6] << 8) | (uint32_t)rx.buf[7]; // B5..B8

	// Communication control parameters
	bool isCommand     = (commCtrl & (uint8_t)CommunicationCtrl::commandBit) == (uint8_t)CommunicationCtrl::commandBit;
	bool isDiscovery   = (commCtrl & (uint8_t)CommunicationCtrl::discoveryBit) == (uint8_t)CommunicationCtrl::discoveryBit;
	bool isPing        = (commCtrl & (uint8_t)CommunicationCtrl::pingBit) == (uint8_t)CommunicationCtrl::pingBit;
	bool isAcknowledge = (commCtrl & (uint8_t)CommunicationCtrl::acknowledgeBit) == (uint8_t)CommunicationCtrl::acknowledgeBit;
	bool isWait        = (commCtrl & (uint8_t)CommunicationCtrl::waitBit) == (uint8_t)CommunicationCtrl::waitBit;
	bool isError       = (commCtrl & (uint8_t)CommunicationCtrl::errorBit) == (uint8_t)CommunicationCtrl::errorBit;
	
	// Data control parameters
	bool isGet        = (dataCtrl & (uint8_t)DataCtrl::operationBits) == (uint8_t)DataCtrl::get;
	bool isSet        = (dataCtrl & (uint8_t)DataCtrl::operationBits) == (uint8_t)DataCtrl::set;
	bool isDelay      = (dataCtrl & (uint8_t)DataCtrl::operationBits) == (uint8_t)DataCtrl::delay;
	bool isListDelays = (dataCtrl & (uint8_t)DataCtrl::operationBits) == (uint8_t)DataCtrl::listDelays;
	bool isExtra      = (dataCtrl & (uint8_t)DataCtrl::operationBits) == (uint8_t)DataCtrl::extra;
	bool isAnalog     = (dataCtrl & (uint8_t)DataCtrl::signalBit) == (uint8_t)DataCtrl::analog;
	bool isDigital    = (dataCtrl & (uint8_t)DataCtrl::signalBit) == (uint8_t)DataCtrl::digital;
	bool isOutput     = (dataCtrl & (uint8_t)DataCtrl::directionBit) == (uint8_t)DataCtrl::output;
	bool isInput      = (dataCtrl & (uint8_t)DataCtrl::directionBit) == (uint8_t)DataCtrl::input;
	bool isBit        = (dataCtrl & (uint8_t)DataCtrl::dataTypeBits) == (uint8_t)DataCtrl::bit;
	bool isByte       = (dataCtrl & (uint8_t)DataCtrl::dataTypeBits) == (uint8_t)DataCtrl::byte;
	bool isInteger    = (dataCtrl & (uint8_t)DataCtrl::dataTypeBits) == (uint8_t)DataCtrl::integer;
	bool isDecimal    = (dataCtrl & (uint8_t)DataCtrl::dataTypeBits) == (uint8_t)DataCtrl::decimal;

	// Config control parameters
	bool isConfig        = (configCtrl & (uint8_t)ConfigCtrl::configBit) == (uint8_t)ConfigCtrl::configBit;
	bool isConfigGet     = (configCtrl & (uint8_t)ConfigCtrl::operationBit) == (uint8_t)ConfigCtrl::get;
	bool isConfigSet     = (configCtrl & (uint8_t)ConfigCtrl::operationBit) == (uint8_t)ConfigCtrl::set;
	uint8_t configOption = (configCtrl & (uint8_t)ConfigCtrl::optionBits);

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
		if (isSet || isPing || isError || isConfig) {
			sendError(from, thisDeviceId, commCtrl, dataCtrl, port, ERR_OPERATION_NOT_ALLOWED);
			return;
		}

		// Return device firmware when asked to identify
		sendAck(from, thisDeviceId, commCtrl, (uint8_t)DataCtrl::integer, 0, FIRMWARE_VERSION);
		return;
	}

	// Ping
	if (isPing) {
		if (isAcknowledge) {
			// Do not answer. Acknowledge sent from another device
			return;
		}
		if (!(rx.id == thisDeviceId || rx.id == CAN_BCAST_ADDRES)) {
			// Frame has to be sent to broadcast address or device address
			return;
		}

		if (isSet || isError || isConfig) {
			sendError(from, thisDeviceId, commCtrl, dataCtrl, port, ERR_OPERATION_NOT_ALLOWED);
			return;
		}

		// Pong
		sendAck(from, thisDeviceId, commCtrl, dataCtrl, 0, 0);
		return;
	}	

	// Only reply to the frames intended for this device
	if (rx.id != thisDeviceId) {
		// Remove delays for other devices
		if (!isConfig && (
				(isCommand && isSet && isOutput && isDigital && !isAcknowledge && !isError)
				|| (!isCommand && isGet && isOutput && isDigital)
			)) {
			removeDelay(rx.id, port);
		}
		
		// From here onward - Only reply to the frames intended for this device
		return;
	}

	// Config
	if (isConfig) {
		// Validate communication byte
		if (!isCommand || isError) {
			sendError(from, thisDeviceId, commCtrl, configCtrl, port, ERR_OPERATION_NOT_ALLOWED);
			return;
		}

		// Write current configuration to EEPROM
		if (isConfigSet == true && static_cast<ConfigOptions>(configOption) == ConfigOptions::writeEEPROM) {
			// Store configuration into EEPROM
			uint32_t EEPROMSize = saveConfig();
			sendAck(from, thisDeviceId, commCtrl, configCtrl, port, EEPROMSize);
			return;
		}

		// Validate port range
		if (port > 15) {
			sendError(from, thisDeviceId, commCtrl, configCtrl, port, ERR_INVALID_PORT);
			return;
		}

		if (isConfigSet) {
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
						if (actionItems[i].inputPort == port) {
							actionItems[i].deviceId = 0xFF;
							actionItems[i].inputPort = 0xFF;
							actionItems[i].mode = ActionMode::normal;
							actionItems[i].type = ActionType::low;
							actionItems[i].ports = 0;
							actionItems[i].delay = 0;
						}
					}
					break;
				case ConfigOptions::actionLow:
					updateActionItem(actionDeviceId, port, ActionMode::normal, ActionType::low, actionPorts);
					break;
				case ConfigOptions::actionHigh:
					updateActionItem(actionDeviceId, port, ActionMode::normal, ActionType::high, actionPorts);
					break;
				case ConfigOptions::actionToggle:
					updateActionItem(actionDeviceId, port, ActionMode::normal, ActionType::toggle, actionPorts);
					break;
				case ConfigOptions::actionLongpressLow:
					updateActionItem(actionDeviceId, port, ActionMode::longpress, ActionType::low, actionPorts);
					break;
				case ConfigOptions::actionLongpressHigh:
					updateActionItem(actionDeviceId, port, ActionMode::longpress, ActionType::high, actionPorts);
					break;
				case ConfigOptions::actionLongpressToggle:
					updateActionItem(actionDeviceId, port, ActionMode::longpress, ActionType::toggle, actionPorts);
					break;
				case ConfigOptions::actionDoubleclickLow:
					updateActionItem(actionDeviceId, port, ActionMode::doubleclick, ActionType::low, actionPorts);
					break;
				case ConfigOptions::actionDoubleclickHigh:
					updateActionItem(actionDeviceId, port, ActionMode::doubleclick, ActionType::high, actionPorts);
					break;
				case ConfigOptions::actionDoubleclickToggle:
					updateActionItem(actionDeviceId, port, ActionMode::doubleclick, ActionType::toggle, actionPorts);
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
					sendError(from, thisDeviceId, commCtrl, configCtrl, port, ERR_OPERATION_NOT_ALLOWED);
					return;
					break;	
			}
			sendAck(from, thisDeviceId, commCtrl, configCtrl, port, data);
			return;
		} else if (isConfigGet) {
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
					// Send configurations for output ports related to all devices on grid
					for (uint16_t i = 0; i < SIZE_ACTION_MAP; i++) {
						if (actionItems[i].inputPort == port) {
							confData = actionItems[i].deviceId << 16 | actionItems[i].ports;
							sendAck(from,
								thisDeviceId,
								commCtrl | (uint8_t)CommunicationCtrl::waitBit,
								(uint8_t)ConfigCtrl::configBit | (uint8_t)actionToConfigType[(uint8_t)actionItems[i].mode][(uint8_t)actionItems[i].type],
								port,
								confData);
							
							if (actionItems[i].delay != 0) {
								// Send information about action delay
								sendAck(from,
									thisDeviceId,
									commCtrl | (uint8_t)CommunicationCtrl::waitBit,
									(uint8_t)ConfigCtrl::configBit | (uint8_t)ConfigOptions::delay,
									port,
									actionItems[i].delay);
							}
						}
					}
					// Send last package as empty
					sendAck(from, thisDeviceId, commCtrl, configCtrl, port, 0);
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
					sendError(from, thisDeviceId, commCtrl, configCtrl, port, ERR_OPERATION_NOT_ALLOWED);
					return;
					break;
			}
			sendAck(from, thisDeviceId, commCtrl, configCtrl, port, confData);
			return;
		}

		// Operation must be Get or Set
		sendError(from, thisDeviceId, commCtrl, configCtrl, port, ERR_OPERATION_NOT_ALLOWED);
		return;
	}

	// Command - data operation
	if (isCommand) {
		if (isGet) {
			// Send back digital input/output value
			if (isBit || isByte || isInteger) {
				sendAck(from, thisDeviceId, commCtrl, dataCtrl, port, isInput ? inputDigitals[port].value : outputDigitals[port].value);
			} else {
				// TODO - TYPE_FLOAT
				sendError(from, thisDeviceId, commCtrl, dataCtrl, port, ERR_UNKNOWN);
			}
			return;
		}

		if (isListDelays) {
			// TODO
			sendError(from, thisDeviceId, commCtrl, dataCtrl, port, ERR_OPERATION_NOT_ALLOWED);
			return;
		}
		
		// Only allow writing to output ports
		if (isInput) {
			sendError(from, thisDeviceId, commCtrl, dataCtrl, port, ERR_OPERATION_NOT_ALLOWED);
			return;
		}

		// Set command
		if (isSet){
			execCommand.isSet = isSet;
			execCommand.isDigital = isDigital;
			execCommand.isAnalog = isAnalog;
			execCommand.isOutput = isOutput;
			execCommand.isBit = isBit;
			execCommand.isByte = isByte;
			execCommand.isInteger = isInteger;
			execCommand.isDecimal = isDecimal;
			execCommand.port = port;
			execCommand.type = (ActionType)data;
		} else if (isDelay) {
			// It has to match on port, digital/analog, input/output otherwise reset
			if (port != execCommand.port || isDigital != execCommand.isDigital || isOutput != execCommand.isOutput) {
				resetCommand(&execCommand);
				sendError(from, thisDeviceId, commCtrl, dataCtrl, port, ERR_OPERATION_NOT_ALLOWED);
				return;
			}
			execCommand.delay = data;
		} else if (isExtra) {
			// It has to match on port, digital/analog, input/output otherwise reset
			if (port != execCommand.port || isDigital != execCommand.isDigital || isOutput != execCommand.isOutput) {
				resetCommand(&execCommand);
				sendError(from, thisDeviceId, commCtrl, dataCtrl, port, ERR_OPERATION_NOT_ALLOWED);
				return;
			}
			// Future use
			execCommand.extra = data;
		}
		
		// Execute command when no other data is expected
		if (!isWait && execCommand.isSet) {
			if (execCommand.delay > 0) {
				setDelay(thisDeviceId, execCommand.port, execCommand.type, execCommand.delay);
			} else {
				// Set output value, for now this covers all dataTypes except decimals
				if (execCommand.isBit || execCommand.isByte || execCommand.isInteger) {
					// Write new value to output port
					setDigitalOutput(execCommand.port, (ActionType)data);
				} else {
					sendError(from, thisDeviceId, commCtrl, dataCtrl, port, ERR_INVALID_TYPE);
					return;
				}
			}
			// Send back current outputDigital value
			sendAck(from, thisDeviceId, commCtrl, (uint8_t)DataCtrl::set, port, outputDigitals[port].value);

			// Reset execCommand properties
			resetCommand(&execCommand);
		}
		return;
	}

	// No action could be taken based on the frame definition
	sendError(from, thisDeviceId, commCtrl, dataCtrl, port, ERR_OPERATION_NOT_ALLOWED);
	return;
}

// Main function to setup stm32
void setup() {
	// Map of actions to config types
	actionToConfigType[(uint8_t)ActionMode::normal][(uint8_t)ActionType::low] = ConfigOptions::actionLow;
	actionToConfigType[(uint8_t)ActionMode::normal][(uint8_t)ActionType::high] = ConfigOptions::actionHigh;
	actionToConfigType[(uint8_t)ActionMode::normal][(uint8_t)ActionType::toggle] = ConfigOptions::actionToggle;
	actionToConfigType[(uint8_t)ActionMode::longpress][(uint8_t)ActionType::low] = ConfigOptions::actionLongpressLow;
	actionToConfigType[(uint8_t)ActionMode::longpress][(uint8_t)ActionType::high] = ConfigOptions::actionLongpressHigh;
	actionToConfigType[(uint8_t)ActionMode::longpress][(uint8_t)ActionType::toggle] = ConfigOptions::actionLongpressToggle;
	actionToConfigType[(uint8_t)ActionMode::doubleclick][(uint8_t)ActionType::low] = ConfigOptions::actionDoubleclickLow;
	actionToConfigType[(uint8_t)ActionMode::doubleclick][(uint8_t)ActionType::high] = ConfigOptions::actionDoubleclickHigh;
	actionToConfigType[(uint8_t)ActionMode::doubleclick][(uint8_t)ActionType::toggle] = ConfigOptions::actionDoubleclickToggle;

	// Compute DIP switches to get device id
	thisDeviceId = computeDeviceAddress();

	// Setup digital inputs
	for (int inputPort = 0; inputPort < SIZE_INPUT_DIGITAL; inputPort++) {
		// external pull-ups/downs as designed
		pinMode(inputDigitalPins[inputPort], INPUT);
		
		// allow signal to settle
		delayMicroseconds(100);

		// Create object
		InputDigital input{};
		input.pin                = inputDigitalPins[inputPort];
		input.value              = digitalRead(inputDigitalPins[inputPort]) == HIGH ? 0 : 1;
		input.debounce           = 0;
		input.processClick       = false;
		input.processDoubleclick = false;
		input.processLongpress   = false;
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
		delay.type     = ActionType::low;
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
	// Take current execution time
	uint64_t loopTime = millis();

	// Calculate loop time
	uint32_t loopTimeDiff = micros() - loopTimeLast;
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
		uint8_t currentValue = digitalRead(inputDigitalPins[inputPort]) == HIGH ? 0 : 1;
		bool inputChanged = false;
		bool bypass = inputConfig[inputPort].bypassInstantly == true
				|| inputConfig[inputPort].bypassOnDisconnect != 0 && loopTime - lastSyncRemote > inputConfig[inputPort].bypassOnDisconnect
				|| dipSwitchBypass && inputConfig[inputPort].bypassOnDIPSwitch == true;

		if (inputConfig[inputPort].debounce == 0) {
			// Without debounce logic
			if (currentValue != inputDigitals[inputPort].value) {
				inputDigitals[inputPort].value = currentValue;
				inputChanged = true;
			}
		} else {
			// With debounce logic
			// Add to debounce counter
			inputDigitals[inputPort].debounce += (currentValue != inputDigitals[inputPort].value ? 1 : -1) * loopTimeDiff;

			// Make sure debounce is not less than zero
			if (inputDigitals[inputPort].debounce < 0) {
				inputDigitals[inputPort].debounce = 0;
			}

			// Convert debounce to logical values
			if (inputDigitals[inputPort].debounce > inputConfig[inputPort].debounce) {
				// Set new value
				inputDigitals[inputPort].value = inputDigitals[inputPort].value == HIGH ? LOW : HIGH;
				inputChanged = true;
				// Reset debounce
				inputDigitals[inputPort].debounce = 0;
			}
		}

		// Push event on input data changed - push raw data
		if (inputChanged) {
			uint8_t commCtrl = (uint8_t)CommunicationCtrl::empty;
			uint8_t dataCtrl = (uint8_t)DataCtrl::input | (uint8_t)DataCtrl::bit;
			// Push to a broadcast address
			canWriteFrame(0xFF, thisDeviceId, commCtrl, dataCtrl, inputPort, inputDigitals[inputPort].value);
		}

		// Record click
		if (inputChanged && (inputConfig[inputPort].isSwitch || (inputConfig[inputPort].isButtonRisingEdge && inputDigitals[inputPort].value == HIGH) || (inputConfig[inputPort].isButtonFallingEdge && inputDigitals[inputPort].value == LOW))) {
			inputDigitals[inputPort].previousClickTime = inputDigitals[inputPort].clickTime;
			inputDigitals[inputPort].clickTime = loopTime;

			// Reset events
			inputDigitals[inputPort].processClick = true;
			inputDigitals[inputPort].processDoubleclick = true;
			inputDigitals[inputPort].processLongpress = true;
		}

		// Bypass actions on double click
		if (inputDigitals[inputPort].processDoubleclick == true && inputDigitals[inputPort].clickTime - inputDigitals[inputPort].previousClickTime < inputConfig[inputPort].doubleclick) {
			// Unset single and double clicks
			inputDigitals[inputPort].processClick = false;
			inputDigitals[inputPort].processDoubleclick = false;
			
			// Only take bypass actions when criteria has been meat
			if (bypass) {
				// Loop actions
				for (uint16_t gridDevIdx = 0; gridDevIdx < SIZE_ACTION_MAP; gridDevIdx++) {
					if (actionItems[gridDevIdx].deviceId != 0xFF && actionItems[gridDevIdx].inputPort == inputPort && actionItems[gridDevIdx].mode == ActionMode::doubleclick) {
						execBypass(gridDevIdx);
					}
				}
			}
		}

		// Bypass actions on single click
		if (inputDigitals[inputPort].processClick == true // Only when click needs processing
			&& (
				inputConfig[inputPort].doubleclick == 0 // Skip when no double click configured
				|| (
					// If still pressed after half of the double click time consider it as single click
					loopTime - inputDigitals[inputPort].clickTime > inputConfig[inputPort].doubleclick * 0.6
					&& (inputConfig[inputPort].isButtonRisingEdge && inputDigitals[inputPort].value == HIGH) || (inputConfig[inputPort].isButtonFallingEdge && inputDigitals[inputPort].value == LOW)
				) || (
					// First click was realy fast, waiting for second click
					loopTime - inputDigitals[inputPort].clickTime > inputConfig[inputPort].doubleclick
				)
			)) {
			inputDigitals[inputPort].processClick = false;
			inputDigitals[inputPort].processDoubleclick = false;
			// Only take bypass actions when criteria has been meat
			if (bypass) {
				// Loop actions
				for (uint16_t gridDevIdx = 0; gridDevIdx < SIZE_ACTION_MAP; gridDevIdx++) {
					if (actionItems[gridDevIdx].deviceId != 0xFF && actionItems[gridDevIdx].inputPort == inputPort && actionItems[gridDevIdx].mode == ActionMode::normal) {
						execBypass(gridDevIdx);
					}
				}
			}
		}

		// Bypass actions on longpress
		if (inputConfig[inputPort].longpress > 0 && inputDigitals[inputPort].processLongpress == true && loopTime - inputDigitals[inputPort].clickTime > inputConfig[inputPort].longpress) {
			inputDigitals[inputPort].processLongpress = false;

			// Only take bypass actions when criteria has been meat
			if (bypass) {
				// Action longpress events
				for (uint16_t gridDevIdx = 0; gridDevIdx < SIZE_ACTION_MAP; gridDevIdx++) {
					if (actionItems[gridDevIdx].deviceId != 0xFF && actionItems[gridDevIdx].inputPort == inputPort && actionItems[gridDevIdx].mode == ActionMode::longpress) {
						execBypass(gridDevIdx);
					}
				}
			}
		}
	}

	// Watch for delay timers
	for (uint8_t delayIdx = 0; delayIdx < SIZE_DELAYS; delayIdx++) {
		if (delays[delayIdx].active == true && delays[delayIdx].time < micros()) {
			if (delays[delayIdx].deviceId == thisDeviceId) {
				setDigitalOutput(delays[delayIdx].port, delays[delayIdx].type);
			} else {
				canWriteFrame(delays[delayIdx].deviceId, thisDeviceId, (uint8_t)CommunicationCtrl::commandBit, (uint8_t)DataCtrl::set, delays[delayIdx].port, (uint8_t)delays[delayIdx].type);
			}
			removeDelay(delays[delayIdx].deviceId, delays[delayIdx].port);
		}
	}
}