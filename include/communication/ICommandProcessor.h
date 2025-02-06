#ifndef I_COMMAND_PROCESSOR_H
#define I_COMMAND_PROCESSOR_H

#include <vector>
#include <cstdint>
#include <string>  // Added for std::string

class ICommandProcessor {
public:
    virtual ~ICommandProcessor() = default;
    
    // Process an input byte
    // Returns true if a complete command is ready
    virtual bool processInput(uint8_t byte) = 0;
    
    // Get the complete command data
    virtual const std::vector<uint8_t>& getCommandData() const = 0;
    
    // Check if a complete command is ready
    virtual bool hasCompleteCommand() const = 0;
    
    // Reset the processor state
    virtual void reset() = 0;
    
    // Get the command code
    virtual const std::string& getCommandCode() const = 0;
};

#endif // I_COMMAND_PROCESSOR_H