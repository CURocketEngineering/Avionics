#ifndef BYTE_COMMAND_PROCESSOR_H
#define BYTE_COMMAND_PROCESSOR_H

#include "ICommandProcessor.h"
#include <vector>
#include <string>

class ByteCommandProcessor : public ICommandProcessor {
public:
    ByteCommandProcessor();
    
    bool processInput(uint8_t byte) override;
    const std::vector<uint8_t>& getCommandData() const override;
    bool hasCompleteCommand() const override;
    void reset() override;
    const std::string& getCommandCode() const override;

private:
    enum class State {
        WAITING_FOR_COMMAND,
        PROCESSING_COMMAND,
        PROCESSING_DATA
    };

    State currentState;
    std::string commandCode;
    std::vector<uint8_t> commandData;
    std::vector<uint8_t> completeData;
    bool commandComplete;

    void processCommandByte(uint8_t byte);
    void processDataByte(uint8_t byte);
};

#endif // BYTE_COMMAND_PROCESSOR_H