#include "communication/ByteCommandProcessor.h"

ByteCommandProcessor::ByteCommandProcessor()
    : currentState(State::WAITING_FOR_COMMAND)
    , commandComplete(false) {
}

bool ByteCommandProcessor::processInput(uint8_t byte) {
    switch (currentState) {
        case State::WAITING_FOR_COMMAND:
            if (byte != 0x00) {  // Not a null terminator
                commandCode.push_back(static_cast<char>(byte));
                currentState = State::PROCESSING_COMMAND;
            }
            break;
            
        case State::PROCESSING_COMMAND:
            if (byte == 0x00) {  // Command code complete
                currentState = State::PROCESSING_DATA;
            } else {
                commandCode.push_back(static_cast<char>(byte));
            }
            break;
            
        case State::PROCESSING_DATA:
            if (byte == 0x00) {  // End of command
                completeData = commandData;  // Store complete data
                commandComplete = true;
                currentState = State::WAITING_FOR_COMMAND;
                return true;
            } else {
                commandData.push_back(byte);
            }
            break;
    }
    
    return false;
}

const std::vector<uint8_t>& ByteCommandProcessor::getCommandData() const {
    return completeData;
}

bool ByteCommandProcessor::hasCompleteCommand() const {
    return commandComplete;
}

void ByteCommandProcessor::reset() {
    currentState = State::WAITING_FOR_COMMAND;
    commandCode.clear();
    commandData.clear();
    completeData.clear();
    commandComplete = false;
}

const std::string& ByteCommandProcessor::getCommandCode() const {
    return commandCode;
}