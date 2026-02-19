#include "UARTCommandHandler.h"
#include "ArduinoHAL.h"
#include <algorithm>

constexpr int COMMAND_CHARS_ASCII_END = 31; // ASCII control characters end at 31, so we can ignore those in input


CommandLine::CommandLine(Stream * UART) : UART(UART) {
}

void CommandLine::begin() {
    help();
    UART->print(SHELL_PROMPT); 
}

static bool isBackspace_(char receivedChar) {
    return receivedChar == static_cast<char>(AsciiKey::Backspace) ||
           receivedChar == static_cast<char>(AsciiKey::Delete);
}

static bool isNewline_(char receivedChar) {
    return receivedChar == '\n' || receivedChar == '\r';
}

namespace {
// Small helper: split on spaces/tabs (no quoting support)
// Parses a line into a command and arguments, e.g. "foo bar baz" -> cmd="foo", args={"bar", "baz"}
void tokenizeWhitespace(const std::string& line,
                              std::string& outCmd,
                              std::queue<std::string>& outArgs)
{
    outCmd.clear();
    while (!outArgs.empty()) {
        outArgs.pop();
    }

    std::string token;
    auto flush = [&]() {
        if (token.empty()) {return;}
        if (outCmd.empty()) {outCmd = token;}
        else {outArgs.push(token);}
        token.clear();
    };

    for (char ch : line) {
        if (ch == ' ' || ch == '\t') {flush();}
        else {token += ch;}
    }
    flush();
}
} // namespace

void CommandLine::readInput() { // NOLINT(readability-function-cognitive-complexity)
    while (UART->available() > 0) {
        const char receivedChar = static_cast<char>(UART->read());

        if (isBackspace_(receivedChar)) {
            handleBackspace_();
        } else if (isNewline_(receivedChar)) {
            if (lastWasCR_){
                lastWasCR_ = false; // Handle \r\n by ignoring the \n
                continue;
            }
            lastWasCR_ = (receivedChar == '\r');
            handleNewline_();
        } else {
            handleChar_(receivedChar);
        }
    }
}

void CommandLine::handleBackspace_() {
    if (fullLine.empty()) {return;}
    fullLine.pop_back();
    UART->print("\b \b"); // erase last char on terminal
}

void CommandLine::handleNewline_() {
    UART->println();

    std::string line = fullLine;
    fullLine.clear();

    trimSpaces(line);
    if (line.empty()) {
        UART->print(SHELL_PROMPT);
        return;
    }

    std::string cmd = {""};
    std::queue<std::string> args = {}; //NOLINT(cppcoreguidelines-init-variables)
    tokenizeWhitespace(line, cmd, args);

    if (!cmd.empty()) {
        executeCommand(cmd, args);
    }

    UART->print(SHELL_PROMPT);
}

void CommandLine::handleChar_(char receivedChar) {
    // Optional: ignore other control chars (keep tab if you want)
    if (static_cast<unsigned char>(receivedChar) <= COMMAND_CHARS_ASCII_END && receivedChar != '\t') {return;}

    if (fullLine.length() >= UART_BUFFER_SIZE - 1) {
        UART->println();
        UART->println("Buffer overflow, input ignored.");
        fullLine.clear();
        UART->print(SHELL_PROMPT);
        return;
    }

    fullLine += receivedChar;
    UART->print(receivedChar);
}


// Add a command with its long name, short name, and function pointer
void CommandLine::addCommand(const std::string& longName, const std::string& shortName, std::function<void(std::queue<std::string>, std::string&)> funcPtr) { //NOLINT(readability-convert-member-functions-to-static)
    Command newCommand{ longName, shortName, funcPtr };
    commands.push_back(newCommand);
}

// Execute a command based on its long name or short name
void CommandLine::executeCommand(const std::string& command, std::queue<std::string> arguments) {
    // Check if the user entered "help" or "?"
    if (command == "help" || command == "?") {
        help();
        return;
    }

    std::string response;
    for (const auto& cmd : commands) {
        if (cmd.longName == command || cmd.shortName == command) {

            cmd.funcPtr(arguments, response);
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

void CommandLine::trimSpaces(std::string& str) { //NOLINT(readability-convert-member-functions-to-static)
    // Remove leading spaces
    size_t start = str.find_first_not_of(" "); //NOLINT(cppcoreguidelines-init-variables)
    // If there's any non-space character
    if (start != std::string::npos) {
        // Remove trailing spaces
        size_t end = str.find_last_not_of(" "); //NOLINT(cppcoreguidelines-init-variables)
        str = str.substr(start, end - start + 1);
    } else {
        // If there are only spaces, clear the string
        str.clear();
    }
}