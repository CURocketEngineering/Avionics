#include "UARTCommandHandler.h"




UartCommandHandler::UartCommandHandler(Stream &port): 
port(port), inCommands(8), outCommands(8){
}

Command UartCommandHandler::popCommand(){
    return inCommands.pop();
}

void UartCommandHandler::pushCommand(Command &command){
    if (outCommands.isFull()){
        Serial.println("outCommands is full, cannot push command");
        return;
    }
    outCommands.push(command);
}

void UartCommandHandler::update(){
    // If there's a command to send, send it (only one to prevent blocking)
    // If there's a command to receive, receive it (only one to prevent blocking)

    // Send a command (only if connected)
    if (!outCommands.isEmpty()){
        Command command = outCommands.pop();
        port.write('<');
        port.write(command.command);
        port.write(command.value);
        port.write('>');
        port.flush();
    }

    // Receive a command
    if (port.available() >= 4){
        Command command;
        port.read(); // Discard the '<'
        command.command = port.read();
        command.value = port.read();
        port.read(); // Discard the '>'
        inCommands.push(command);
    }

    // Handle the commands
    while (!inCommands.isEmpty()){
        Command command = inCommands.pop();
        switch (command.command){
            case CURE_PING:
                handlePing(command);
                break;
            default:
                break;
        }
    }
}

bool UartCommandHandler::handlePing(Command &command){
    if (command.command == CURE_PING){
        Command response;
        response.command = CURE_PING;
        response.value = 0;
        pushCommand(response);
        return true;
    }
    return false;
}