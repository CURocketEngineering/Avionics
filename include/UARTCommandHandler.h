#ifndef UART_COMMAND_HANDLER_H
#define UART_COMMAND_HANDLER_H

#include "ArduinoHAL.h"
#include "CommandNames.h"
#include "data_handling/CircularArray.h"

#include <stdint.h>
#include <stdbool.h>

struct Command {
  uint8_t command;
  uint8_t value;
};

class UartCommandHandler {
  public:
    // Construct with the usb-port to use, however, it should match Arduino's default Serial for sending debug info
    UartCommandHandler(Stream &port);

    void update(); // Update the command handler, checks for new commands and sends out the queued ones
    
    Command popCommand(); // Get the oldest unread command that's been received
    void pushCommand(Command &command); // Send a command out the uart

  private:
    Stream& port;  
    
    // Circular Arrays for storing in and out commands
    CircularArray<Command> inCommands;
    CircularArray<Command> outCommands;

    // All the different command handlers
    // Returns true if the command was handled
    bool handlePing(Command &command);
};

#endif