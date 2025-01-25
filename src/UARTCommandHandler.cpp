#include "UARTCommandHandler.h"
#include "ArduinoHAL.h"
#include <algorithm>

CommandLine::CommandLine(Stream * UART) : UART(UART) {
}

void CommandLine::begin() {
    help();
    UART->print("CL> "); 
}


void CommandLine::readInput() {

    while (UART->available() > 0) {
        char incomingByte = UART->read(); 

        // Handle the Up Arrow key (ASCII 27 + [ + ([A] || [B]))
        if (incomingByte == 27) {  // ASCII ESC (start of escape sequence)
            // Read the escape sequence buffer
            delay(2);
            incomingByte = UART->read();  
            if(incomingByte == '[')
            {
                delay(2);
                incomingByte = UART->read(); 
                // Check if the full sequence matches an arrow key
                if (incomingByte == 'A') {
                    // Up Arrow
                    if (historyIndex > 0) {
                        historyIndex--;
                    }
                    displayCommandFromHistory();
                } else if (incomingByte == 'B') {
                    // Down Arrow
                    if (historyIndex < MAX_HISTORY) {
                        historyIndex++;
                    }
                    displayCommandFromHistory();
                } else {
                    // Handle other escape sequences or ignore unknown ones
                    UART->println("Unknown escape sequence: ");
                }
            }
        }
        else if (incomingByte == 8 || incomingByte == 127)  // 8 = Backspace, 127 = Delete
        { 
            if (!fullLine.empty()) {
                fullLine.pop_back();  // Remove the last character from the buffer
                UART->print("\b \b");  // Move cursor back, print a space, and move cursor back again
                
                if(!inputBuffer.empty()){
                    inputBuffer.pop_back(); 
                }
                //TODO FIX: If you back space on a history command it will not erase
                //the command or argument buffer, so if you press enter it will
                // return prevous commmand
            }
        }
        else if (incomingByte == ' ')
        {
            UART->print(incomingByte);
            fullLine += incomingByte;

            if (!inputBuffer.empty()) {
                if (!isCommandParsed) {
                    command = inputBuffer;
                    isCommandParsed = true;
                } else {
                    if(argSize[historyIndex] < MAX_ARGUMENTS){
                        argumentQueue.push(inputBuffer);  // Push subsequent words as arguments
                        argSize[historyIndex]++;
                    }
                }
            }
            inputBuffer.clear();
        }
        else if (incomingByte == '\n' || incomingByte == '\r') 
        {
            UART->println();
            int idx= 0;
            // If it's a newline, process the current buffer and add to history
            if (!inputBuffer.empty()) {
                if(isCommandParsed || !command.empty())
                {   
                    argumentQueue.push(inputBuffer);
                    argSize[historyIndex]++;
                }
                else
                {
                    command = inputBuffer;
                }
            }

            if(!command.empty() || !argumentQueue.empty())
            {
                executeCommand(command, argumentQueue);
                commandHistory[historyIndex] = command;
                fullLineHistory[historyIndex] = fullLine;

                while (!argumentQueue.empty()) {
                    argumentHistory[historyIndex][idx] = argumentQueue.front();
                    argumentQueue.pop();  // Clear the argument queue
                    idx++;
                }
                historyIndex++;
            }

            inputBuffer.clear();  // Clear the buffer for the next input
            command.clear();
            fullLine.clear();
            isCommandParsed = false;
            UART->print("CL> "); 

            
        } else {
            // Add the incoming byte to the buffer, and update the UART display
            if (inputBuffer.length() < UART_BUFFER_SIZE - 1) {  // Ensure we don't overflow the buffer
                inputBuffer += incomingByte;
                fullLine += incomingByte;
                UART->print(incomingByte);  // Echo the character to the terminal

            } else {
                UART->println("Buffer overflow, input ignored.");
            }
        }

    }
}

void CommandLine::displayCommandFromHistory() {

    if (historyIndex >= MAX_HISTORY) {
        // Reset the history index to 0 (wrap around) and clear the history buffer
        historyIndex = 0;  // Reset to the beginning of history

        // Optionally clear the history buffer or handle how you want to reset the entries
        // For example, clearing all argumentHistory buffers:
        for (int i = 0; i < MAX_HISTORY; i++) {
            for (int idx = 0; idx < MAX_ARGUMENTS; idx++) {
                argumentHistory[i][idx] = "";
            }
            commandHistory[i] = "";
            fullLineHistory[i] = "";
        }
    }

    if (historyIndex >= 0 && historyIndex < MAX_HISTORY) {
        // Clear current input and display the previous/next command from history
        inputBuffer.clear(); 
        command.clear();
        fullLine.clear();
        while (!argumentQueue.empty()) {
            argumentQueue.front();
            argumentQueue.pop();  
        }

        fullLine = fullLineHistory[historyIndex];
        command = commandHistory[historyIndex];
        for (int idx = 0; idx < argSize[historyIndex]; idx++)  {
            argumentQueue.push(argumentHistory[historyIndex][idx]);
        }

        UART->print("\r");  // Move the cursor to the beginning of the line

        for(int len = 0; len < 40; len++)
            UART->print(" ");  // Clear the current line (clear any previous characters)


        UART->print("\r");  // Move back to the beginning of the line
        UART->print("CL> "); 
        UART->print(fullLine.c_str());  // Print the command from history
    } 


}

// Add a command with its long name, short name, and function pointer
void CommandLine::addCommand(const string& longName, const string& shortName, function<void(queue<string>, string&)> funcPtr) {
    Command newCommand{ longName, shortName, funcPtr };
    commands.push_back(newCommand);
}

// Execute a command based on its long name or short name
void CommandLine::executeCommand(const string& command, queue<string> arugments) {
    // Check if the user entered "help" or "?"
    if (command == "help" || command == "?") {
        help();
        return;
    }

    string response;
    for (const auto& cmd : commands) {
        if (cmd.longName == command || cmd.shortName == command) {

            cmd.funcPtr(arugments, response);
            return;
        }
    }

   UART->println("Command not found");

}

// Help function to list all commands with their long and short names
void CommandLine::help(){
    if (commands.empty()) {
        UART->println("No commands available.");  
        UART->println("help<?>"); 
        return;
    }
 
    for (const auto& cmd : commands) {
        UART->println(String(cmd.longName.c_str()) + "<" + String(cmd.shortName.c_str()) + ">");  
    }
    UART->println("help<?>"); 

}

void CommandLine::trimSpaces(std::string& str) {
    // Remove leading spaces
    size_t start = str.find_first_not_of(" ");
    // If there's any non-space character
    if (start != std::string::npos) {
        // Remove trailing spaces
        size_t end = str.find_last_not_of(" ");
        str = str.substr(start, end - start + 1);
    } else {
        // If there are only spaces, clear the string
        str.clear();
    }
}