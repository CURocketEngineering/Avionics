#include "communication/CommandHandler.h"
#include "communication/ByteCommandProcessor.h"

CommandHandler::CommandHandler() 
    : processor(std::make_unique<ByteCommandProcessor>()) {
}

void CommandHandler::bindCommand(const std::string& commandCode, HandlerFunc handler) {
    handlers[commandCode] = handler;
}

bool CommandHandler::processCommandByte(uint8_t byte) {
    // If we have an active sub-handler, delegate to it
    if (!activeHandlers.empty()) {
        bool result = activeHandlers.top()->processCommandByte(byte);
        if (result) {
            lastResponse = activeHandlers.top()->getResponse();
            // If the sub-handler completed its task, remove it
            if (!activeHandlers.top()->hasActiveHandler()) {
                activeHandlers.pop();
            }
        }
        return result;
    }

    // Process byte through our command processor
    if (processor->processInput(byte)) {
        executeCommand();
        return true;
    }
    return false;
}

void CommandHandler::executeCommand() {
    const std::string& commandCode = processor->getCommandCode();
    auto it = handlers.find(commandCode);
    
    if (it != handlers.end()) {
        lastResponse = it->second(processor->getCommandData());
    } else {
        lastResponse = {0xFF};  // Error response
    }
    
    processor->reset();
}

const std::vector<uint8_t>& CommandHandler::getResponse() const {
    return lastResponse;
}

void CommandHandler::pushHandler(SubCommandHandler handler) {
    activeHandlers.push(handler);
}

void CommandHandler::popHandler() {
    if (!activeHandlers.empty()) {
        activeHandlers.pop();
    }
}

bool CommandHandler::hasActiveHandler() const {
    return !activeHandlers.empty();
}