#include <FlashEEPROM.h>
#include <IWatchdog.h>
#include "STM32_CAN.h"
#include "STM32F103Rxv2.h"

// Firmware
// Version is defined as 00 <Mayor> <Minor> <Bugfix>
#define FIRMWARE_VERSION  0x00010203UL

// Communication control byte
enum class CommCtrl: uint8_t {
	empty          = 0x00,
	discoveryBit   = 0x80,
	pingBit        = 0x40,
	acknowledgeBit = 0x20,
	errorBit       = 0x10,
	waitBit        = 0x08,
	notifyBit      = 0x04,
};

// Data control
enum class DataCtrl: uint8_t {
	empty         = 0x00,
	commandBit    = 0x80,
	configBit     = 0x40,
	signalBit     = 0x20,
	directionBit  = 0x10,
	dataTypeBit   = (0x08+0x04),
	// Specific Types
	digital       = 0x00,
	analog        = 0x20,
	output        = 0x00,
	input         = 0x10,
	bit           = 0x00,
	byte          = 0x04,
	integer       = 0x08,
	decimal       = 0x0C,
};

// Command operations
enum class CommandOper: uint8_t {
	empty             = 0x00,
	get               = 0x00,
	set               = 0x01,
	delay             = 0x02,
	listDelays        = 0x03,
	clearDelayById    = 0x04,
	clearDelayByPort  = 0x05,
};

// Config operations
enum class ConfigOper: uint8_t {
	get                 = 0x00, // Combine get operation with the rest of the operations
	set                 = 0x80, // Combine set operation with the rest of the operations
	
	empty               = 0x00,
	debounce            = 0x01, // Debounce in microseconds
	doubleclick         = 0x02, // Double-click in milliseconds
	actions             = 0x03, // Get/Reset all actions
	actionBase          = 0x04, // Action P1 deviceId (B5), trigger (B6), mode (B7), type (B8)
	actionPorts         = 0x05, // Action P2 ports (map)
	actionSkipWhenDelay = 0x06, // Action P3 skip action if delay is present in any of the output ports (map)
	actionClearDelays   = 0x07, // Action P4 clear all delays on all specified output ports (map)
	actionDelay         = 0x08, // Action P5 delay in milliseconds
	actionLongpress     = 0x09, // Action P6 longpress in milliseconds
	actionConfigSwitch  = 0x0A, // Action P7 Config switch
	bypassInstantly     = 0x0B, // Bypass Instantly
	bypassOnDisconnect  = 0x0C, // Bypass on disconnect in milliseconds
	
	writeEEPROM         = 0x7F, // Write all configuration into EEPROM
};

// Action types
enum class ActionTrigger: uint8_t {
	disabled     = 0x00, // No trigger
	rising       = 0x01, // Trigger on input port - Rising edge
	falling      = 0x02, // Trigger on input port - Falling edge
};

// Action types
enum class ActionType: uint8_t {
	low          = LOW,
	high         = HIGH,
	toggle       = 0b010,
	pwm          = 0b110,
};

enum class ActionMode: uint8_t {
	click       = 0b00,
	longpress   = 0b01,
	doubleclick = 0b10,
};

// Sizes
#define SIZE_DEVICE_ADDRESS  5
#define SIZE_CONFIG_SWITCH   4
#define SIZE_INPUT_DIGITAL   16
#define SIZE_INPUT_ANALOG    4
#define SIZE_OUTPUT_DIGITAL  12
#define SIZE_ACTION_MAP      256
#define SIZE_DELAYS          128
#define SIZE_COMMANDS        64

// Error codes (packed in Data on error/ack)
#define ERR_UNKNOWN               0x00000001UL
#define ERR_OPERATION_NOT_ALLOWED 0x00000002UL
#define ERR_CONFIG_NOT_ALLOWED    0x00000003UL
#define ERR_INVALID_TYPE          0x00000004UL
#define ERR_INVALID_PORT          0x00000005UL
#define ERR_WRONG_ADDRESS         0x00000006UL

// CAN setup
STM32_CAN Can1(CAN_RX, CAN_TX, RX_SIZE_256, TX_SIZE_256);

#define CAN_BROADCAST_ADDRES 0xFF // broadcast frame receiver

// Maximum lifetime of a command before the command is deleted
#define COMMAND_MAX_AGE 500

uint32_t firmwareVersion;

// Logical device identity
uint8_t thisDeviceId = 0;

// Hardware pin maps (from your original sketch)
const int deviceAddressPins[] = { DEV_A1, DEV_A2, DEV_A3, DEV_A4, DEV_A5 };
const int inputDigitalPins[]  = { DI_01, DI_02, DI_03, DI_04, DI_05, DI_06, DI_07, DI_08, DI_09, DI_10, DI_11, DI_12, DI_13, DI_14, DI_15, DI_16 };
const int outputDigitalPins[] = { DO_01, DO_02, DO_03, DO_04, DO_05, DO_06, DO_07, DO_08, DO_09, DO_10, DO_11, DO_12 };
const int configurationPins[] = { C_01, C_02, C_03, C_04 };


struct InputDigital {
	uint8_t pin;
	uint8_t value;
	int32_t debounce;
	bool bypass;
};
InputDigital inputDigitals[SIZE_INPUT_DIGITAL];

struct OutputDigital {
	uint8_t pin;
	uint8_t value;
};
OutputDigital outputDigitals[SIZE_OUTPUT_DIGITAL];

struct Delay {
	uint32_t id;
	bool active;
	bool execute;
	uint8_t deviceId;
	uint8_t port;
	ActionType type;
	int64_t time;
};
Delay delays[SIZE_DELAYS];

struct ActionItem {
	uint8_t inputPort;
	ActionTrigger trigger;
	ActionType type;
	ActionMode mode;
	uint8_t skipWhenDelayDeviceId;
	uint16_t skipWhenDelayPorts;
	uint8_t clearDelayDeviceId;
	uint16_t clearDelayPorts;
	uint8_t deviceId;
	uint16_t ports;
	uint32_t delay;
	uint32_t longpress;
	uint8_t configSwitch;
	// Internal tracking information
	bool processClick;
	bool processDoubleclick;
	bool processLongpress;
	uint64_t clickTime;
	uint64_t previousClickTime;
};
struct ActionItemEEPROM {
	uint8_t inputPort;
	ActionTrigger trigger;
	ActionType type;
	ActionMode mode;
	uint8_t skipWhenDelayDeviceId;
	uint16_t skipWhenDelayPorts;
	uint8_t clearDelayDeviceId;
	uint16_t clearDelayPorts;
	uint8_t deviceId;
	uint16_t ports;
	uint32_t delay;
	uint32_t longpress;
	uint8_t configSwitch;
};

struct Command {
	bool active;
	uint32_t packageId;
	uint32_t commandTime;
	bool isNotify;
	// Command properties
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
	int32_t debounce; // trigger in microseconds
	int32_t doubleclick; // trigger in microseconds
	bool bypassInstantly;
	int32_t bypassOnDisconnect; // bypess after x miliseconds from last ping
};

// Set configuration for each input pin
ConfigRegister inputConfig[SIZE_INPUT_DIGITAL];
ActionItem actionItems[SIZE_ACTION_MAP];
ActionItem* tempActionItem = nullptr;
Command commands[SIZE_COMMANDS];

// Global variables
uint64_t loopTimeLast = 0;

// Last time in milliseconds
int32_t lastSyncRemote = 0;

// Last delay id
// TODO implement increment function
uint32_t delayIdSequence = 1;

// Device package sequence
uint32_t commandIdSequence = 0;

// Helpers to pack 32-bit data (MSB..LSB)
static inline void u32ToBytes(uint32_t source, uint8_t* target) {
	target[0] = (uint8_t)(source >> 24);
	target[1] = (uint8_t)(source >> 16);
	target[2] = (uint8_t)(source >> 8);
	target[3] = (uint8_t)(source);
}

// Compute Device ID from Address DIP Switch
static inline uint8_t computeDeviceAddress() {
	uint8_t id = 0;
	for (int pin = 0; pin < SIZE_DEVICE_ADDRESS; pin++) {
		pinMode(deviceAddressPins[pin], INPUT_PULLUP);
		// LOW means switch ON -> bit=1
		id |= ((digitalRead(deviceAddressPins[pin]) == LOW) ? 1 : 0) << pin;
	}
	return id;
}

// Compute Config DIP Switch
static inline uint8_t computeConfigSwitch() {
	uint8_t configSwitch = 0;
	for (int pin = 0; pin < SIZE_CONFIG_SWITCH; pin++) {
		// LOW means switch ON -> bit=1
		configSwitch |= ((digitalRead(configurationPins[pin]) == LOW) ? 1 : 0) << pin;
	}
	return configSwitch;
}

static inline uint32_t nextPackageId(uint8_t initiatorId, uint8_t responderId) {
	if (commandIdSequence > 0x1FFF) {
		commandIdSequence == 0;
	}
	uint32_t packageId = commandIdSequence << 16 | initiatorId << 8 | responderId;
	commandIdSequence++;
	return packageId;
}

// Send CAN frame
void canWriteFrame(uint32_t packageId, uint8_t commCtrl, uint8_t dataCtrl, uint8_t operation, uint8_t port, uint32_t data) {
	CAN_message_t tx{};
	tx.flags.extended = 1;  // Enable extended ID.
	tx.id  = packageId;
	tx.len = 8;

	tx.buf[0] = commCtrl;         // B1 CommCtrl
	tx.buf[1] = dataCtrl;         // B2 DataCtrl
	tx.buf[2] = operation;        // B3 Operation
	tx.buf[3] = port;             // B4 Port
	u32ToBytes(data, &tx.buf[4]); // B5..B8 Data MSB..LSB

	Can1.write(tx);
}

// Send acknowledge frame
void sendAck(uint32_t packageId, uint8_t commCtrl, uint8_t dataCtrl, uint8_t operation, uint8_t port, uint32_t data) {
	// Set acknowledgeBit, ensure no errorBit
	canWriteFrame(
		packageId,
		(commCtrl | (uint8_t)CommCtrl::acknowledgeBit) & ~(uint8_t)CommCtrl::errorBit,
		dataCtrl,
		operation,
		port,
		data);
}

// Send error frame
void sendError(uint32_t packageId, uint8_t commCtrl, uint8_t dataCtrl, uint8_t operation, uint8_t port, uint32_t errCode) {
	// Set acknowledgeBit and errorBit
	canWriteFrame(
		packageId,
		(commCtrl | (uint8_t)CommCtrl::acknowledgeBit) & (uint8_t)CommCtrl::errorBit,
		dataCtrl,
		operation,
		port,
		errCode);
}

// Change status of the output port
void setDigitalOutput(uint8_t port, ActionType actionType) {
	if (port < SIZE_OUTPUT_DIGITAL) {
		// Calculate new value
		uint32_t value;

		// Set new value
		if (actionType == ActionType::toggle){
			// Toggle current value
			value = outputDigitals[port].value == HIGH ? LOW : HIGH;
		} else if(actionType == ActionType::low) {
			// Set new value as LOW
			value = LOW;
		} else if(actionType == ActionType::high) {
			// Set new value as HIGH
			value = HIGH;
		}

		// If value has changed, push data change frame
		if (outputDigitals[port].value != value){
			uint8_t commCtrl  = (uint8_t)CommCtrl::empty;
			uint8_t dataCtrl  = (uint8_t)DataCtrl::bit;
			uint8_t operation = (uint8_t)CommandOper::empty;
			canWriteFrame(nextPackageId(thisDeviceId, 0xFF), commCtrl, dataCtrl, operation, port, value);

			// Do the actuall change
			digitalWrite(outputDigitalPins[port], value);
			outputDigitals[port].value = value;
		}
	}
}

void setDigitalOutputRemote(uint8_t deviceId, uint8_t port, ActionType actionType) {
	// Send command to change remote output port
	canWriteFrame(nextPackageId(thisDeviceId, deviceId), (uint8_t)CommCtrl::empty, (uint8_t)DataCtrl::commandBit, (uint8_t)CommandOper::set, port, (uint8_t)actionType);
}

void setDelay(uint8_t deviceId, uint8_t port, ActionType type, uint32_t delay, bool execute = true) {
	for (uint8_t delayIdx = 0; delayIdx < SIZE_DELAYS; delayIdx++) {
		if (!delays[delayIdx].active) {
			delays[delayIdx].id = delayIdSequence++;
			delays[delayIdx].active = true;
			delays[delayIdx].execute = execute;
			delays[delayIdx].deviceId = deviceId;
			delays[delayIdx].port = port;
			delays[delayIdx].type = type;
			delays[delayIdx].time = micros() + delay * 1000;
			break;
		}
	}
}

std::vector<uint32_t> clearDelays(uint8_t deviceId, uint8_t port) {
	std::vector<uint32_t> deletedDelayIds;
	deletedDelayIds.reserve(SIZE_DELAYS);

	for (uint8_t delayIdx = 0; delayIdx < SIZE_DELAYS; delayIdx++) {
		if (delays[delayIdx].deviceId == deviceId && delays[delayIdx].port == port) {
			// Stack removed delay
			deletedDelayIds.push_back(delays[delayIdx].id);

			// Remove delay
			delays[delayIdx].id = 0;
			delays[delayIdx].active = false;
			delays[delayIdx].execute = false;
			delays[delayIdx].deviceId = 0xFF;
			delays[delayIdx].port = 0;
			delays[delayIdx].type = ActionType::low; 
			delays[delayIdx].time = 0;
		}
	}
	return deletedDelayIds;
}

bool clearDelayById(uint8_t deviceId, uint id) {
	for (uint8_t delayIdx = 0; delayIdx < SIZE_DELAYS; delayIdx++) {
		if (delays[delayIdx].deviceId == deviceId && delays[delayIdx].id == id && delays[delayIdx].active) {
			delays[delayIdx].id = 0;
			delays[delayIdx].active = false;
			delays[delayIdx].execute = false;
			delays[delayIdx].deviceId = 0xFF;
			delays[delayIdx].port = 0;
			delays[delayIdx].type = ActionType::low; 
			delays[delayIdx].time = 0;
			return true;
		}
	}
	return false;
}

void resetActionItem(uint16_t idx) {
	actionItems[idx].deviceId              = 0xFF;
	actionItems[idx].inputPort             = 0xFF;
	actionItems[idx].trigger               = ActionTrigger::disabled;
	actionItems[idx].mode                  = ActionMode::click;
	actionItems[idx].type                  = ActionType::low;
	actionItems[idx].skipWhenDelayDeviceId = 0xFF;
	actionItems[idx].skipWhenDelayPorts    = 0;
	actionItems[idx].clearDelayDeviceId    = 0xFF;
	actionItems[idx].clearDelayPorts       = 0;
	actionItems[idx].ports                 = 0;
	actionItems[idx].delay                 = 0;
	actionItems[idx].longpress             = 0;
	actionItems[idx].configSwitch          = 0;
	actionItems[idx].processClick          = false;
	actionItems[idx].processDoubleclick    = false;
	actionItems[idx].processLongpress      = false;
	actionItems[idx].clickTime             = 0;
	actionItems[idx].previousClickTime     = 0;
}

void resetConfig() {
	// Reset input configuration
	for (uint8_t inputPort = 0; inputPort < SIZE_INPUT_DIGITAL; inputPort++) {
		ConfigRegister config{};
		config.debounce            = 0;
		config.doubleclick         = 0;
		config.bypassInstantly     = false;
		config.bypassOnDisconnect  = 0;
		inputConfig[inputPort] = config;
	}

	// Reset actions
	for (uint16_t idx = 0; idx < SIZE_ACTION_MAP; idx++){
		resetActionItem(idx);
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
		// Only store important properties into EEPROM
		ActionItemEEPROM actionItemEEPROM;
		actionItemEEPROM.deviceId              = actionItems[i].deviceId;
		actionItemEEPROM.inputPort             = actionItems[i].inputPort;
		actionItemEEPROM.trigger               = actionItems[i].trigger;
		actionItemEEPROM.type                  = actionItems[i].type;
		actionItemEEPROM.mode                  = actionItems[i].mode;
		actionItemEEPROM.skipWhenDelayDeviceId = actionItems[i].skipWhenDelayDeviceId;
		actionItemEEPROM.skipWhenDelayPorts    = actionItems[i].skipWhenDelayPorts;
		actionItemEEPROM.clearDelayDeviceId    = actionItems[i].clearDelayDeviceId;
		actionItemEEPROM.clearDelayPorts       = actionItems[i].clearDelayPorts;
		actionItemEEPROM.ports                 = actionItems[i].ports;
		actionItemEEPROM.delay                 = actionItems[i].delay;
		actionItemEEPROM.longpress             = actionItems[i].longpress;
		actionItemEEPROM.configSwitch          = actionItems[i].configSwitch;

		EEPROM.put(EEPROMPointer, actionItemEEPROM);
		EEPROMPointer += sizeof(actionItemEEPROM);
	}

	// Commit changes
	EEPROM.commit();

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
		ActionItemEEPROM actionItemEEPROM;
		EEPROM.get(EEPROMPointer, actionItemEEPROM);
		EEPROMPointer += sizeof(actionItemEEPROM);

		// Copy ActionItemEEPROM object to ActionItem
		actionItems[i].deviceId              = actionItemEEPROM.deviceId;
		actionItems[i].inputPort             = actionItemEEPROM.inputPort;
		actionItems[i].trigger               = actionItemEEPROM.trigger;
		actionItems[i].type                  = actionItemEEPROM.type;
		actionItems[i].mode                  = actionItemEEPROM.mode;
		actionItems[i].skipWhenDelayDeviceId = actionItemEEPROM.skipWhenDelayDeviceId;
		actionItems[i].skipWhenDelayPorts    = actionItemEEPROM.skipWhenDelayPorts;
		actionItems[i].clearDelayDeviceId    = actionItemEEPROM.clearDelayDeviceId;
		actionItems[i].clearDelayPorts       = actionItemEEPROM.clearDelayPorts;
		actionItems[i].ports                 = actionItemEEPROM.ports;
		actionItems[i].delay                 = actionItemEEPROM.delay;
		actionItems[i].longpress             = actionItemEEPROM.longpress;
		actionItems[i].configSwitch          = actionItemEEPROM.configSwitch;
	}
}

void updateActionItem(uint8_t inputPort, ActionTrigger trigger, ActionMode mode, ActionType type) {
	for (uint16_t idx = 0; idx < SIZE_ACTION_MAP; idx++) {
		if (actionItems[idx].deviceId == 0xFF) {
			resetActionItem(idx);
			actionItems[idx].inputPort = inputPort;
			actionItems[idx].trigger = trigger;
			actionItems[idx].mode = mode;
			actionItems[idx].type = type;
			tempActionItem = &actionItems[idx];
			break;
		}
	}
}

void updateActionPorts(uint8_t deviceId, uint16_t ports) {
	tempActionItem->deviceId = deviceId;
	tempActionItem->ports = ports;
}

void updateActionSkipWhenDelay(uint8_t deviceId, uint16_t ports) {
	tempActionItem->skipWhenDelayDeviceId = deviceId;
	tempActionItem->skipWhenDelayPorts = ports;
}

void updateActionClearDelays(uint8_t deviceId, uint16_t ports) {
	tempActionItem->clearDelayDeviceId = deviceId;
	tempActionItem->clearDelayPorts = ports;
}

void updateActionDelay(uint32_t delay) {
	tempActionItem->delay = delay;
}

void updateActionLongpress(uint32_t longpress) {
	tempActionItem->longpress = longpress;
}

void updateActionConfigSwitch(uint8_t configSwitch) {
	tempActionItem->configSwitch = configSwitch;
}

Command& getCommand(uint32_t packageId){
	// Check for existing command
	for (uint8_t idx = 0; idx < SIZE_COMMANDS; idx++) {
		if (commands[idx].active == true && commands[idx].packageId == packageId) {
			commands[idx].commandTime = millis();
			return commands[idx];
		}
	}
	
	// Reserve a slot in the stack of commands
	for (uint8_t idx = 0; idx < SIZE_COMMANDS; idx++) {
		if (commands[idx].active == false) {
			commands[idx] = Command{};
			commands[idx].commandTime = millis();
			commands[idx].packageId = packageId;
			commands[idx].active = true;
			return commands[idx];
		}
	}

	// Override oldest command
	uint8_t oldestIdx = 0;
	for (uint8_t idx = 0; idx < SIZE_COMMANDS; idx++) {
		if (commands[idx].commandTime < commands[oldestIdx].commandTime) {
			oldestIdx = idx;
		}
	}
	commands[oldestIdx] = Command{};
	commands[oldestIdx].commandTime = millis();
	commands[oldestIdx].packageId = packageId;
	commands[oldestIdx].active = true;
	return commands[oldestIdx];
}

void removeCommand(Command& command) {
	command.packageId = 0;
	command.commandTime     = 0;
	command.isNotify        = false;
	command.active          = false;
	command.isSet           = false;
	command.isDigital       = false;
	command.isAnalog        = false;
	command.isOutput        = false;
	command.isBit           = false;
	command.isByte          = false;
	command.isInteger       = false;
	command.isDecimal       = false;
	command.port            = 0xFF;
	command.type            = ActionType::low;
	command.delay           = 0;
	command.extra           = 0;
}

void maintainCommands() {
	// Remove old commands
	for (uint8_t idx = 0; idx < SIZE_COMMANDS; idx++) {
		if (millis() - commands[idx].commandTime > COMMAND_MAX_AGE) {
			removeCommand(commands[idx]);
		}
	}
}

void execBypass(uint8_t gridDevIdx) {
	// Remove delays
	if (actionItems[gridDevIdx].clearDelayPorts) {
		for (uint8_t outputPort = 0; outputPort < SIZE_OUTPUT_DIGITAL; outputPort++) {
			if ((actionItems[gridDevIdx].clearDelayPorts & (1 << outputPort)) > 0) {
				clearDelays(actionItems[gridDevIdx].clearDelayDeviceId, outputPort);

				// Notify other devices to clear the delay
				uint8_t commCtrl = (uint8_t)CommCtrl::notifyBit;
				uint8_t dataCtrl = (uint8_t)DataCtrl::commandBit | (uint8_t)DataCtrl::output;
				uint8_t operation = (uint8_t)CommandOper::clearDelayByPort;
				// Send Notify package to others
				canWriteFrame(nextPackageId(thisDeviceId, actionItems[gridDevIdx].clearDelayDeviceId), commCtrl, dataCtrl, operation, outputPort, 0);
			}
		}
	}
	for (uint8_t outputPort = 0; outputPort < SIZE_OUTPUT_DIGITAL; outputPort++) {
		if (actionItems[gridDevIdx].ports & (1 << outputPort)) {
			if (actionItems[gridDevIdx].delay > 0) {
				// Change value of local output port
				setDelay(actionItems[gridDevIdx].deviceId, outputPort, actionItems[gridDevIdx].type, actionItems[gridDevIdx].delay);

				// Notify other devices to set the informational delay
				uint8_t commCtrl = (uint8_t)CommCtrl::notifyBit;
				uint8_t dataCtrl = (uint8_t)DataCtrl::commandBit | (uint8_t)DataCtrl::output;
				uint8_t operation = (uint8_t)CommandOper::empty;
				// Send Notify package to others
				uint32_t delayPackageId = nextPackageId(thisDeviceId, actionItems[gridDevIdx].deviceId);
				canWriteFrame(delayPackageId, commCtrl | (uint8_t)CommCtrl::waitBit, dataCtrl, operation | (uint8_t)CommandOper::set, outputPort, (uint8_t)actionItems[gridDevIdx].type);
				canWriteFrame(delayPackageId, commCtrl, dataCtrl, operation | (uint8_t)CommandOper::delay, outputPort, actionItems[gridDevIdx].delay);
			} else if (actionItems[gridDevIdx].deviceId == thisDeviceId) {
				// Change value of local output port
				setDigitalOutput(outputPort, actionItems[gridDevIdx].type);
			} else {
				setDigitalOutputRemote(actionItems[gridDevIdx].deviceId, outputPort, actionItems[gridDevIdx].type);
			}
		}
	}
}

// Handle one received CAN frame for us
void canProcessFrame(const CAN_message_t& rx) {
	// Only process frames where CAN messages where length is 8 bytes
	if (rx.len != 8) return;

	uint32_t packageId       = rx.id;
	uint16_t commandId       = rx.id >> 16;
	uint8_t initiatorId     = (rx.id & 0xFF00) >> 8;
	uint8_t responderId     = rx.id & 0xFF;

	// Unpack CAN Payload
	uint8_t commCtrl   = rx.buf[0];        // B1
	uint8_t dataCtrl   = rx.buf[1];        // B2
	uint8_t operation  = rx.buf[2];        // B3
	uint8_t port       = rx.buf[3];        // B4
	uint32_t data      = ((uint32_t)rx.buf[4] << 24) | ((uint32_t)rx.buf[5] << 16) | ((uint32_t)rx.buf[6] << 8) | (uint32_t)rx.buf[7]; // B5..B8

	// Communication control parameters
	bool isDiscovery   = (commCtrl & (uint8_t)CommCtrl::discoveryBit) == (uint8_t)CommCtrl::discoveryBit;
	bool isPing        = (commCtrl & (uint8_t)CommCtrl::pingBit) == (uint8_t)CommCtrl::pingBit;
	bool isAcknowledge = (commCtrl & (uint8_t)CommCtrl::acknowledgeBit) == (uint8_t)CommCtrl::acknowledgeBit;
	bool isError       = (commCtrl & (uint8_t)CommCtrl::errorBit) == (uint8_t)CommCtrl::errorBit;
	bool isWait        = (commCtrl & (uint8_t)CommCtrl::waitBit) == (uint8_t)CommCtrl::waitBit;
	bool isNotify      = (commCtrl & (uint8_t)CommCtrl::notifyBit) == (uint8_t)CommCtrl::notifyBit;
	
	// Data control parameters
	bool isCommand    = (dataCtrl & (uint8_t)DataCtrl::commandBit) == (uint8_t)DataCtrl::commandBit;
	bool isConfig     = (dataCtrl & (uint8_t)DataCtrl::configBit) == (uint8_t)DataCtrl::configBit;
	bool isDigital    = (dataCtrl & (uint8_t)DataCtrl::signalBit) == (uint8_t)DataCtrl::digital;
	bool isAnalog     = (dataCtrl & (uint8_t)DataCtrl::signalBit) == (uint8_t)DataCtrl::analog;
	bool isOutput     = (dataCtrl & (uint8_t)DataCtrl::directionBit) == (uint8_t)DataCtrl::output;
	bool isInput      = (dataCtrl & (uint8_t)DataCtrl::directionBit) == (uint8_t)DataCtrl::input;
	// Data types
	bool isBit        = (dataCtrl & (uint8_t)DataCtrl::dataTypeBit) == (uint8_t)DataCtrl::bit;
	bool isByte       = (dataCtrl & (uint8_t)DataCtrl::dataTypeBit) == (uint8_t)DataCtrl::byte;
	bool isInteger    = (dataCtrl & (uint8_t)DataCtrl::dataTypeBit) == (uint8_t)DataCtrl::integer;
	bool isDecimal    = (dataCtrl & (uint8_t)DataCtrl::dataTypeBit) == (uint8_t)DataCtrl::decimal;

	// Discovery
	if (isDiscovery) {
		if (isAcknowledge) {
			// Do not answer. Acknowledge sent from another device
			return;
		}
		if (responderId != CAN_BROADCAST_ADDRES) {
			// Frame has to be sent to broadcast address
			return;
		}
		if (isPing || isError || isConfig) {
			sendError(packageId, commCtrl, dataCtrl, operation, port, ERR_OPERATION_NOT_ALLOWED);
			return;
		}

		// Return device firmware when asked to identify
		sendAck(packageId, commCtrl, dataCtrl, operation, port, thisDeviceId);
		return;
	}

	// Ping
	if (isPing) {
		if (isAcknowledge) {
			// Do not answer. Acknowledge sent from another device
			return;
		}
		if (!(responderId == thisDeviceId || responderId == CAN_BROADCAST_ADDRES)) {
			// Frame has to be sent to broadcast address or device address
			return;
		}

		if (isError || isConfig) {
			sendError(packageId, commCtrl, dataCtrl, operation, port, ERR_OPERATION_NOT_ALLOWED);
			return;
		}

		// Pong
		sendAck(packageId, commCtrl, dataCtrl, operation, 0, thisDeviceId);
		return;
	}	

	// Config
	if (isConfig) {
		bool isConfigGet         = (operation & 0x80) == (uint8_t)ConfigOper::get;
		bool isConfigSet         = (operation & 0x80) == (uint8_t)ConfigOper::set;
		uint32_t configOper = operation & 0x7F;

		// Only process if this device is expected to be a responder)
		if (responderId != thisDeviceId){
			return;
		}
		
		// Validate communication byte
		if (isAcknowledge || isError) {
			sendError(packageId, commCtrl, dataCtrl, operation, port, ERR_OPERATION_NOT_ALLOWED);
			return;
		}

		// Write current configuration to EEPROM
		if (isConfigSet && static_cast<ConfigOper>(configOper) == ConfigOper::writeEEPROM) {
			// Store configuration into EEPROM
			uint32_t EEPROMSize = saveConfig();
			sendAck(packageId, commCtrl, dataCtrl, operation, port, EEPROMSize);
			return;
		}

		// Validate port range
		if (port > 15) {
			sendError(packageId, commCtrl, dataCtrl, operation, port, ERR_INVALID_PORT);
			return;
		}

		if (isConfigSet) {
			switch (static_cast<ConfigOper>(configOper)) {
				case ConfigOper::debounce:
					inputConfig[port].debounce = data;
					break;
				case ConfigOper::doubleclick:
					inputConfig[port].doubleclick = data;
					break;
				case ConfigOper::actions:
					// Remove all actions for specific port
					for (uint16_t idx = 0; idx < SIZE_ACTION_MAP; idx++) {
						if (actionItems[idx].inputPort == port) {
							resetActionItem(idx);
						}
					}
					break;
				case ConfigOper::actionBase: {
					ActionTrigger trigger = (ActionTrigger)((data >> 16) & 0xFF);
					ActionMode mode = (ActionMode)((data >> 8) & 0xFF);
					ActionType type = (ActionType)(data & 0xFF);
					updateActionItem(port, trigger, mode, type);
					break;
				}
				case ConfigOper::actionPorts: {
					uint8_t actionDeviceId = data >> 24;
					uint16_t actionPorts = data & 0xFFF;
					updateActionPorts(actionDeviceId, actionPorts);
					break;
				}
				case ConfigOper::actionSkipWhenDelay: {
					uint16_t actionDeviceId = data >> 24;
					uint16_t actionPorts = data & 0xFFF;
					updateActionSkipWhenDelay(actionDeviceId, actionPorts);
					break;
				}
				case ConfigOper::actionClearDelays: {
					uint8_t actionDeviceId = data >> 24;
					uint16_t actionPorts = data & 0xFFF;
					updateActionClearDelays(actionDeviceId, actionPorts);
					break;
				}
				case ConfigOper::actionDelay:
					updateActionDelay(data);
					break;
				case ConfigOper::actionLongpress:
					updateActionLongpress(data);
					break;
				case ConfigOper::actionConfigSwitch:
					updateActionConfigSwitch(data);
					break;
				case ConfigOper::bypassInstantly:
					inputConfig[port].bypassInstantly = data > 0;
					break;
				case ConfigOper::bypassOnDisconnect:
					inputConfig[port].bypassOnDisconnect = data;
					break;
				default:
					sendError(packageId, commCtrl, dataCtrl, operation, port, ERR_OPERATION_NOT_ALLOWED);
					return;
					break;	
			}
			sendAck(packageId, commCtrl, dataCtrl, operation, port, data);
			return;
		} else if (isConfigGet) {
			uint32_t confData = 0;
			switch (static_cast<ConfigOper>(configOper)) {
				case ConfigOper::debounce:
					confData = ((uint32_t)inputConfig[port].debounce);
					break;
				case ConfigOper::doubleclick:
					confData = ((uint32_t)inputConfig[port].doubleclick);
					break;
				case ConfigOper::actions:
					ConfigOper typeToActionConf[9];
					// Send configurations for output ports related to all devices on grid
					for (uint16_t i = 0; i < SIZE_ACTION_MAP; i++) {
						if (actionItems[i].inputPort == port) {
							confData = ((uint8_t)actionItems[i].trigger) << 16 | ((uint8_t)actionItems[i].mode) << 8 | (uint8_t)actionItems[i].type;
							// P1 - action base
							sendAck(packageId,
								commCtrl | (uint8_t)CommCtrl::waitBit,
								(uint8_t)DataCtrl::configBit,
								(uint8_t)ConfigOper::actionBase,
								port,
								confData);
							
							// P2 - action ports
							sendAck(packageId,
								commCtrl | (uint8_t)CommCtrl::waitBit,
								(uint8_t)DataCtrl::configBit,
								(uint8_t)ConfigOper::actionPorts,
								port,
								(actionItems[i].deviceId << 24) | actionItems[i].ports);

							// P3 - action skip when delay
							sendAck(packageId,
								commCtrl | (uint8_t)CommCtrl::waitBit,
								(uint8_t)DataCtrl::configBit,
								(uint8_t)ConfigOper::actionSkipWhenDelay,
								port,
								(actionItems[i].skipWhenDelayDeviceId << 24) | actionItems[i].skipWhenDelayPorts);

							// P4 - clear delays
							sendAck(packageId,
								commCtrl | (uint8_t)CommCtrl::waitBit,
								(uint8_t)DataCtrl::configBit,
								(uint8_t)ConfigOper::actionClearDelays,
								port,
								(actionItems[i].clearDelayDeviceId << 24) | actionItems[i].clearDelayPorts);
									
							// P5 - action delay
							sendAck(packageId,
								commCtrl | (uint8_t)CommCtrl::waitBit,
								(uint8_t)DataCtrl::configBit,
								(uint8_t)ConfigOper::actionDelay,
								port,
								actionItems[i].delay);
							
							// P6 - action longpress in milliseconds
							sendAck(packageId,
								commCtrl | (uint8_t)CommCtrl::waitBit,
								(uint8_t)DataCtrl::configBit,
								(uint8_t)ConfigOper::actionLongpress,
								port,
								actionItems[i].longpress);
							
							// P7 - action config switch
							sendAck(packageId,
								commCtrl | (uint8_t)CommCtrl::waitBit,
								(uint8_t)DataCtrl::configBit,
								(uint8_t)ConfigOper::actionConfigSwitch,
								port,
								actionItems[i].configSwitch);
						}
					}
					// Send empty package without waitBit
					sendAck(packageId, commCtrl, dataCtrl, operation, port, 0);
					return;
				case ConfigOper::bypassInstantly:
					confData = inputConfig[port].bypassInstantly ? 1 : 0;
					break;
				case ConfigOper::bypassOnDisconnect:
					confData = inputConfig[port].bypassOnDisconnect;
					break;
				default:
					sendError(packageId, commCtrl, dataCtrl, operation, port, ERR_OPERATION_NOT_ALLOWED);
					return;
					break;
			}
			sendAck(packageId, commCtrl, dataCtrl, operation, port, confData);
			return;
		}

		// Operation must be Get or Set
		sendError(packageId, commCtrl, dataCtrl, operation, port, ERR_OPERATION_NOT_ALLOWED);
		return;
	}

	// Command - data operation
	if (isCommand) {
		bool isGet              = operation == (uint8_t)CommandOper::get;
		bool isSet              = operation == (uint8_t)CommandOper::set;
		bool isDelay            = operation == (uint8_t)CommandOper::delay;
		bool isListDelays       = operation == (uint8_t)CommandOper::listDelays;
		bool isClearDelayById   = operation == (uint8_t)CommandOper::clearDelayById;
		bool isClearDelayByPort = operation == (uint8_t)CommandOper::clearDelayByPort;

		if (isClearDelayById) {
			bool delayCleard = false;
			if (port == 0xFF) {
				// Only process when thisDeviceId was a target
				if (responderId == thisDeviceId) {
					delayCleard = clearDelayById(thisDeviceId, data);
				}
			}

			// Only respond to initiator when thisDeviceId was a target
			if (responderId == thisDeviceId) {
				if (delayCleard) {
					sendAck(packageId, commCtrl, dataCtrl, operation, port, delayCleard);
				} else {
					sendError(packageId, commCtrl, dataCtrl, operation, port, ERR_UNKNOWN);
				}
			}
			return;
		}

		if (isClearDelayByPort) {
			// Clear delays for a specific port
			std::vector<uint32_t> deletedDelayIds = clearDelays(responderId, port);
			
			// This operation could not return error
			if (responderId == thisDeviceId) {
				// Only respond to initiator when thisDeviceId was a target
				for (uint32_t deletedDelayId : deletedDelayIds) {
					sendAck(packageId, commCtrl | (uint8_t)CommCtrl::waitBit, dataCtrl, operation, port, deletedDelayId);
				}
				// Final acknowleadge
				sendAck(packageId, commCtrl, dataCtrl, operation, port, 0);
			}
			return;
		}

		if (isGet) {
			// Only respond to initiator when thisDeviceId was a target
			if (responderId == thisDeviceId) {
				// Send back digital input/output value
				if (isBit || isByte || isInteger) {
					sendAck(packageId, commCtrl, dataCtrl, operation, port, isInput ? inputDigitals[port].value : outputDigitals[port].value);
				} else {
					// TODO - TYPE_FLOAT
					sendError(packageId, commCtrl, dataCtrl, operation, port, ERR_UNKNOWN);
				}
			}
			return;
		}

		if (isListDelays) {
			// Only process when thisDeviceId was a target
			if (responderId == thisDeviceId) {
				// List all delays
				for (uint8_t delayIdx = 0; delayIdx < SIZE_DELAYS; delayIdx++) {
					if (delays[delayIdx].active == true) {
						dataCtrl = (uint8_t)DataCtrl::digital | (uint8_t)DataCtrl::output | (uint8_t)DataCtrl::integer;
						sendAck(packageId, commCtrl | (uint8_t)CommCtrl::waitBit, dataCtrl, operation, delays[delayIdx].port, delays[delayIdx].id);
						sendAck(packageId, commCtrl | (uint8_t)CommCtrl::waitBit, dataCtrl, operation, delays[delayIdx].port, delays[delayIdx].deviceId);
						sendAck(packageId, commCtrl | (uint8_t)CommCtrl::waitBit, dataCtrl, operation, delays[delayIdx].port, delays[delayIdx].execute == true ? 1 : 0);
						sendAck(packageId, commCtrl | (uint8_t)CommCtrl::waitBit, dataCtrl, operation, delays[delayIdx].port, (uint8_t)delays[delayIdx].type);
						sendAck(packageId, commCtrl | (uint8_t)CommCtrl::waitBit, dataCtrl, operation, delays[delayIdx].port, (delays[delayIdx].time - micros())/1000);
					}
				}
				// Send last package without wait and empty
				sendAck(packageId, commCtrl, dataCtrl, operation, 0xFF, 0);
			}
			return;
		}
		
		// Only allow writing to output ports
		if (isInput) {
			// Only respond when thisDeviceId was a target
			if (responderId == thisDeviceId) {
				sendError(packageId, commCtrl, dataCtrl, operation, port, ERR_OPERATION_NOT_ALLOWED);
			}
			return;
		}

		// Set command
		Command& command = getCommand(packageId);
		if (isSet){
			command.isNotify = isNotify;
			command.isSet = isSet;
			command.isDigital = isDigital;
			command.isAnalog = isAnalog;
			command.isInput = isInput;
			command.isOutput = isOutput;
			command.isBit = isBit;
			command.isByte = isByte;
			command.isInteger = isInteger;
			command.isDecimal = isDecimal;
			command.port = port;
			command.delay = 0;
			command.extra = 0;
			command.type = (ActionType)data;
		} else if (isDelay) {
			// Command has to match on port, digital/analog, input/output otherwise remove command
			if (port != command.port || isDigital != command.isDigital || isOutput != command.isOutput) {
				removeCommand(command);
				sendError(packageId, commCtrl, dataCtrl, operation, port, ERR_OPERATION_NOT_ALLOWED);
				return;
			}
			command.delay = data;
		}
		
		// Add informational delay for other devices
		if (responderId != thisDeviceId) {
			if (!isWait && command.isSet && command.delay > 0) {
				bool execute = command.isNotify == true ? false : true;
				setDelay(responderId, command.port, command.type, command.delay, execute);
			}
			return;
		}
			
		// Execute command when no other data is expected
		if (!isWait && command.isSet) {
			if (command.delay > 0) {
				bool execute = command.isNotify == true ? false : true;
				setDelay(responderId, command.port, command.type, command.delay, execute);
			} else {
				// Set output value, for now this covers all dataTypes except decimals
				if (command.isBit || command.isByte || command.isInteger) {
					// Write new value to output port
					setDigitalOutput(command.port, command.type);
				} else {
					// Cleanup and respond with an error
					removeCommand(command);
					sendError(packageId, commCtrl, dataCtrl, operation, port, ERR_INVALID_TYPE);
					return;
				}
			}
			// Send back current outputDigital value
			sendAck(packageId, commCtrl, dataCtrl, operation, port, outputDigitals[port].value);

			// Remove command from cache
			removeCommand(command);
		}
		return;
	}

	// Only respond to initiator when this device was marked as a responder
	if (responderId == thisDeviceId) {
		// No action could be taken based on the frame definition
		sendError(packageId, commCtrl, dataCtrl, operation, port, ERR_OPERATION_NOT_ALLOWED);
	}
	return;
}

// Main function to setup stm32
void setup() {
	// Compute Address DIP switch to get device id
	thisDeviceId = computeDeviceAddress();

	// Setup power led
	pinMode(POWER_LED, OUTPUT);

	// Setup digital inputs
	for (int inputPort = 0; inputPort < SIZE_INPUT_DIGITAL; inputPort++) {
		// external pull-ups/downs as designed
		pinMode(inputDigitalPins[inputPort], INPUT);
		
		// allow signal to settle
		delayMicroseconds(100);

		// Create object
		InputDigital input{};
		input.pin      = inputDigitalPins[inputPort];
		input.value    = digitalRead(inputDigitalPins[inputPort]) == HIGH ? 0 : 1;
		input.debounce = 0;
		input.bypass   = false;
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
		delay.id       = 0;
		delay.active   = false;
		delay.execute   = false;
		delay.deviceId = 0xFF;
		delay.port     = 0;
		delay.type     = ActionType::low;
		delay.time     = 0;
		delays[delayIdx] = delay;
	}

	// Setup configuration pins
	for (int pin = 0; pin < SIZE_CONFIG_SWITCH; pin++) {
		pinMode(configurationPins[pin], INPUT_PULLUP);
	}

	// Must call this to read first page
	EEPROM.begin();

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
	uint32_t canBaudRate = 500000;

	// Initialize CAN, enable retransmission and set baudrate 
	Can1.begin(true);
	Can1.setBaudRate(canBaudRate);

	// Initialize the IWDG with 5 seconds timeout.
	// This would cause a CPU reset if the IWDG timer
	// is not reloaded in approximately 5 seconds.
	IWatchdog.begin(5000000);
}

// Loop indefinetely
void loop() {
	// Take current execution time
	uint64_t loopTime = millis();

	// Calculate loop time
	uint32_t loopTimeDiff = micros() - loopTimeLast;
	loopTimeLast = micros();

	// Blink to show process is running
	digitalWrite(POWER_LED, (loopTime / 1000) % 2 == 0 ? true : false);

	// Read and process CAN messages
	CAN_message_t rx;
	while (Can1.read(rx)) {
		canProcessFrame(rx);
	}

	// Compute config switch every loop
	uint8_t configSwitch = computeConfigSwitch();

	// Scan inputs and detect changes (for potential push events/actions)
	for (int inputPort = 0; inputPort < SIZE_INPUT_DIGITAL; inputPort++) {
		uint8_t currentValue = digitalRead(inputDigitalPins[inputPort]) == HIGH ? LOW : HIGH;
		bool inputChanged = false;
		
		// Calculate bypass criteria
		inputDigitals[inputPort].bypass = inputConfig[inputPort].bypassInstantly == true
				|| inputConfig[inputPort].bypassOnDisconnect != 0 && loopTime - lastSyncRemote > inputConfig[inputPort].bypassOnDisconnect;

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
				inputDigitals[inputPort].value = currentValue;
				inputChanged = true;
				// Reset debounce
				inputDigitals[inputPort].debounce = 0;
			}
		}

		// Push event on input data changed - push raw data
		if (inputChanged) {
			uint8_t commCtrl  = (uint8_t)CommCtrl::empty;
			uint8_t dataCtrl  = (uint8_t)DataCtrl::input | (uint8_t)DataCtrl::bit;
			uint8_t operation = (uint8_t)CommandOper::empty;
			// Push to a broadcast address
			canWriteFrame(nextPackageId(thisDeviceId, 0xFF), commCtrl, dataCtrl, operation, inputPort, inputDigitals[inputPort].value);

			// Only take bypass actions when bypass criteria have been met
			if (inputDigitals[inputPort].bypass) {
				// Loop actions
				for (uint16_t gridDevIdx = 0; gridDevIdx < SIZE_ACTION_MAP; gridDevIdx++) {
					// Action configSwitch required
					if (!(actionItems[gridDevIdx].configSwitch == 0 || configSwitch & (1 << (actionItems[gridDevIdx].configSwitch - 1)))) {
						continue;
					}
					if (actionItems[gridDevIdx].deviceId != 0xFF && actionItems[gridDevIdx].inputPort == inputPort) {
						// Make sure trigger matches action trigger
						if ((actionItems[gridDevIdx].trigger == ActionTrigger::rising && inputDigitals[inputPort].value == HIGH) || (actionItems[gridDevIdx].trigger == ActionTrigger::falling && inputDigitals[inputPort].value == LOW)){
							// Set timeings
							actionItems[gridDevIdx].previousClickTime = actionItems[gridDevIdx].clickTime;
							actionItems[gridDevIdx].clickTime = loopTime;

							// Skip action if delay is set for a specific output
							bool skipAction = false;
							for (uint8_t delayIdx = 0; delayIdx < SIZE_DELAYS; delayIdx++) {
								if (delays[delayIdx].active == true && delays[delayIdx].execute == true
									&& actionItems[gridDevIdx].skipWhenDelayDeviceId == delays[delayIdx].deviceId
									&& (actionItems[gridDevIdx].skipWhenDelayPorts & (1 << delays[delayIdx].port)) > 0) {
									skipAction = true;
									break;
								}
							}

							if (!skipAction){
								// Set click markers
								actionItems[gridDevIdx].processClick = true;
								actionItems[gridDevIdx].processDoubleclick = true;
	
								// Set longpress markers
								if (actionItems[gridDevIdx].mode == ActionMode::longpress && actionItems[gridDevIdx].longpress > 0) {
									actionItems[gridDevIdx].processLongpress = true;
								}
							}
							
						}
					}
				}
			}
		}
	}

	for (uint16_t gridDevIdx = 0; gridDevIdx < SIZE_ACTION_MAP; gridDevIdx++) {
		// Action configSwitch required
		if (!(actionItems[gridDevIdx].configSwitch == 0 || configSwitch & (1 << (actionItems[gridDevIdx].configSwitch - 1)))) {
			continue;
		}
		
		// Action validity and input bypass
		if (actionItems[gridDevIdx].deviceId == 0xFF || !inputDigitals[actionItems[gridDevIdx].inputPort].bypass) {
			continue;
		}
		
		// Bypass actions on double click
		if (actionItems[gridDevIdx].processDoubleclick == true
			&& actionItems[gridDevIdx].mode == ActionMode::doubleclick
			&& actionItems[gridDevIdx].clickTime - actionItems[gridDevIdx].previousClickTime < inputConfig[actionItems[gridDevIdx].inputPort].doubleclick
		) {
			// Unset click markers
			actionItems[gridDevIdx].processDoubleclick = false;
			// Only take bypass actions when criteria has been meat
			execBypass(gridDevIdx);

			// Unset marker for action items with click mode with the same trigger type
			for (uint16_t i = 0; i < SIZE_ACTION_MAP; i++) {
				if (actionItems[i].deviceId != 0xFF
					&& actionItems[i].inputPort == actionItems[gridDevIdx].inputPort
					&& actionItems[i].trigger == actionItems[gridDevIdx].trigger
					&& actionItems[i].mode == ActionMode::click
				) {
					actionItems[i].processClick = false;
				}	
			}
		}

		// Bypass actions on single click
		if (actionItems[gridDevIdx].processClick == true
			&& actionItems[gridDevIdx].mode == ActionMode::click
		) { // Only when click needs processing
			if (inputConfig[actionItems[gridDevIdx].inputPort].doubleclick == 0 // No double click configured - continue to process click
				|| (
					// Wait full time, possible double click
					loopTime - actionItems[gridDevIdx].clickTime > inputConfig[actionItems[gridDevIdx].inputPort].doubleclick
				)
			) {
				actionItems[gridDevIdx].processClick = false;
				execBypass(gridDevIdx);

				// Unset marker for action items with double click mode with the same trigger type
				for (uint16_t i = 0; i < SIZE_ACTION_MAP; i++) {
					if (actionItems[i].deviceId != 0xFF
						&& actionItems[i].inputPort == actionItems[gridDevIdx].inputPort
						&& actionItems[i].trigger == actionItems[gridDevIdx].trigger
						&& actionItems[i].mode == ActionMode::doubleclick
					) {
						actionItems[i].processDoubleclick = false;
					}	
				}
			}
		}

		// Bypass actions on longpress
		if (actionItems[gridDevIdx].processLongpress == true
			&& actionItems[gridDevIdx].mode == ActionMode::longpress
			&& (
				(actionItems[gridDevIdx].trigger == ActionTrigger::rising && inputDigitals[actionItems[gridDevIdx].inputPort].value == HIGH)
				||
				(actionItems[gridDevIdx].trigger == ActionTrigger::falling && inputDigitals[actionItems[gridDevIdx].inputPort].value == LOW)
			) && loopTime - actionItems[gridDevIdx].clickTime > actionItems[gridDevIdx].longpress
		) {
			actionItems[gridDevIdx].processLongpress = false;
			execBypass(gridDevIdx);
		}
	}

	// Watch for delay timers
	for (uint8_t delayIdx = 0; delayIdx < SIZE_DELAYS; delayIdx++) {
		if (delays[delayIdx].active == true && delays[delayIdx].time < micros()) {
			if (delays[delayIdx].execute == true) {
				if (delays[delayIdx].deviceId == thisDeviceId) {
					setDigitalOutput(delays[delayIdx].port, delays[delayIdx].type);
				} else {
					setDigitalOutputRemote(delays[delayIdx].deviceId, delays[delayIdx].port, delays[delayIdx].type);
				}
			}
			// Clear out only this delay
			clearDelayById(delays[delayIdx].deviceId, delays[delayIdx].id);
		}
	}

	// Reload watchdog timer
	IWatchdog.reload();

	// Maintain commands
	maintainCommands();
}