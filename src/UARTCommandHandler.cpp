#include "UARTCommandHandler.h"

// Constructor: Initializes the CommandLine object
CommandLine::CommandLine() {
    UART.begin(115200);
}

void CommandLine::begin() {
    help();
    UART.print("CL> "); 
}


// Function to read UART input byte by byte, with support for arrow and backspace
void CommandLine::readInput() {
    while (UART.available() > 0) {
        char incomingByte = UART.read();  // Read the incoming byte

        // Handle the Up Arrow key (ASCII 27 + [A])
        if (incomingByte == 27) {  // ASCII ESC (start of escape sequence)
            if (UART.available() > 0 && UART.read() == '[') {
                if (UART.available() > 0) {
                    char arrowKey = UART.read();  // Read the arrow key (A = up, B = down)
                    if (arrowKey == 'A') {
                        // Up Arrow: Show the previous command from history
                        if (historyIndex > 0) {
                            historyIndex--;
                        }

                        displayCommandFromHistory();

                    } else if (arrowKey == 'B') {
                        // Down Arrow: Show the next command in history (if needed)
                        if (historyIndex < commandHistory.size() - 1) {
                            historyIndex++;  // Move to the next command in history
                        }
                        displayCommandFromHistory();  // Display the command
                    }
                    
                }
            }
        }
        // Handle Backspace key
        else if (incomingByte == 8 || incomingByte == 127) {  // 8 = Backspace, 127 = Delete
            if (!inputBuffer.empty()) {
                inputBuffer.pop_back();  // Remove the last character from the buffer
                UART.print("\b \b");  // Move cursor back, print a space, and move cursor back again
            }
        }
        // Handle normal character input
        else if (incomingByte == '\n' || incomingByte == '\r') {
            // If it's a newline, process the current buffer and add to history
            if (!inputBuffer.empty()) {
                processCommand(inputBuffer);
                commandHistory.push_back(inputBuffer);  // Add command to history
                historyIndex = commandHistory.size();  // Reset history index
            }
            inputBuffer = "";  // Clear the buffer for the next input
            UART.println();  // Move to a new line
            UART.print("CL> "); 
        } else {
            // Add the incoming byte to the buffer, and update the UART display
            if (inputBuffer.length() < UART_BUFFER_SIZE - 1) {  // Ensure we don't overflow the buffer
                inputBuffer += incomingByte;
                UART.print(incomingByte);  // Echo the character to the terminal
            } else {
                // Buffer overflow warning
                UART.println("Buffer overflow, input ignored.");
            }
        }
    }
}

void CommandLine::displayCommandFromHistory() {
    if (historyIndex >= 0 && historyIndex < commandHistory.size()) {
        // Clear current input and display the previous/next command from history
        inputBuffer = commandHistory[historyIndex];
        UART.print("\r");  // Move the cursor to the beginning of the line
        UART.print("                ");  // Clear the current line (clear any previous characters)
        UART.print("\r");  // Move back to the beginning of the line
        UART.print(inputBuffer.c_str());  // Print the command from history
    }
}

// Function to process the command in the buffer
void CommandLine::processCommand(const std::string& command) {
    executeCommand(command);
}

// Add a command with its long name, short name, and function pointer
void CommandLine::addCommand(const std::string& longName, const std::string& shortName, std::function<void(const std::string&, const std::string&)> funcPtr) {
    Command newCommand{ longName, shortName, funcPtr };
    commands.push_back(newCommand);
}

// Execute a command based on its long name or short name
void CommandLine::executeCommand(const std::string& command) {
    // Check if the user entered "help" or "?"
    if (command == "help" || command == "?") {
        help();
        return;
    }

    // Search for the command in the list
    for (const auto& cmd : commands) {
        if (cmd.longName == command || cmd.shortName == command) {
            // Need to change LongName to arugments you have after 
            // command name
            //shortname will be a response that people can recieved
            //not necesarry
            cmd.funcPtr(cmd.longName, cmd.shortName);
            return;
        }
    }

   UART.println("\nCommand not found");

}

// Help function to list all commands with their long and short names
void CommandLine::help(){
    if (commands.empty()) {
        UART.println("\nNo commands available.");  
        return;
    }

    UART.println("\nAvailable commands:");  
    for (const auto& cmd : commands) {
        UART.println(String(cmd.longName.c_str()) + "<" + String(cmd.shortName.c_str()) + ">");  
    }
}