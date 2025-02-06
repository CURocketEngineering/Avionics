#ifndef COMMAND_HANDLER_H
#define COMMAND_HANDLER_H

#include "ICommandProcessor.h"
#include <functional>
#include <map>
#include <string>
#include <vector>
#include <memory>
#include <stack>

class CommandHandler {
public:
    // Function type for command handlers - takes raw command bytes and returns response bytes
    using HandlerFunc = std::function<std::vector<uint8_t>(const std::vector<uint8_t>&)>;
    using SubCommandHandler = std::shared_ptr<CommandHandler>;
    
    CommandHandler();
    
    // Add a command handler for a specific command code sequence
    void bindCommand(const std::string& commandCode, HandlerFunc handler);
    
    // Process an incoming command byte
    // Returns true if a complete command was processed
    bool processCommandByte(uint8_t byte);
    
    // Get the response bytes for the last processed command
    const std::vector<uint8_t>& getResponse() const;

    // Push a sub-command handler onto the stack
    void pushHandler(SubCommandHandler handler);

    // Pop the current sub-command handler
    void popHandler();

    // Check if there's an active sub-handler
    bool hasActiveHandler() const;

private:
    std::unique_ptr<ICommandProcessor> processor;
    std::map<std::string, HandlerFunc> handlers;
    std::stack<SubCommandHandler> activeHandlers;
    std::vector<uint8_t> lastResponse;
    
    // Process a complete command
    void executeCommand();
};

#endif // COMMAND_HANDLER_H