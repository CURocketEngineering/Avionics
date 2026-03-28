#include "UARTCommandHandler.h"
#include "ArduinoHAL.h"
#include <algorithm>

constexpr int kCommandCharsAsciiEnd = 31; // ASCII control characters end at 31, so we can ignore those in input


CommandLine::CommandLine(Stream* uartStream) : uart_(uartStream), defaultUart_(uartStream) {
}

void CommandLine::switchUART(Stream* newUart) {
    if (newUart == nullptr) {
        return;
    }
    uart_ = newUart;
}

void CommandLine::useDefaultUART() {
    uart_ = defaultUart_;
}

void CommandLine::begin() {
    help();
    uart_->print(kShellPrompt);
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
    bool consumedInputThisCall = false;

    while (uart_->available() > 0) {
        consumedInputThisCall = true;
        const char receivedChar = static_cast<char>(uart_->read());

        if (isBackspace_(receivedChar)) {
            lastWasCR_ = false;
            handleBackspace_();
        } else if (isNewline_(receivedChar)) {
            if (lastWasCR_ && receivedChar == '\n') {
                lastWasCR_ = false; // Treat CRLF as a single newline event.
                continue;
            }
            lastWasCR_ = (receivedChar == '\r');
            handleNewline_();
        } else {
            lastWasCR_ = false;
            handleChar_(receivedChar);
        }
    }

    if (consumedInputThisCall) {
        lastInteractionTimestamp_ = static_cast<uint32_t>(millis());
    }
}

void CommandLine::handleBackspace_() {
    if (fullLine_.empty()) {return;}
    fullLine_.pop_back();
    uart_->print("\b \b"); // erase last char on terminal
}

void CommandLine::handleNewline_() {
    uart_->println();

    std::string line = fullLine_;
    fullLine_.clear();

    trimSpaces(line);
    if (line.empty()) {
        uart_->print(kShellPrompt);
        return;
    }

    std::string cmd = {""};
    std::queue<std::string> args = {}; //NOLINT(cppcoreguidelines-init-variables)
    tokenizeWhitespace(line, cmd, args);

    if (!cmd.empty()) {
        executeCommand(cmd, args);
    }

    uart_->print(kShellPrompt);
}

void CommandLine::handleChar_(char receivedChar) {
    // Optional: ignore other control chars (keep tab if you want)
    if (static_cast<unsigned char>(receivedChar) <= kCommandCharsAsciiEnd && receivedChar != '\t') {return;}

    if (fullLine_.length() >= kUartBufferSize - 1) {
        uart_->println();
        uart_->println("Buffer overflow, input ignored.");
        fullLine_.clear();
        uart_->print(kShellPrompt);
        return;
    }

    fullLine_ += receivedChar;
    uart_->print(receivedChar);
}


// Add a command with its long name, short name, and function pointer
void CommandLine::addCommand(const std::string& longName, const std::string& shortName, std::function<void(std::queue<std::string>, std::string&)> funcPtr) { //NOLINT(readability-convert-member-functions-to-static)
    Command newCommand{ longName, shortName, funcPtr };
    commands_.push_back(newCommand);
}

// Execute a command based on its long name or short name
void CommandLine::executeCommand(const std::string& command, std::queue<std::string> arguments) {
    // Check if the user entered "help" or "?"
    if (command == "help" || command == "?") {
        help();
        return;
    }

    std::string response;
    for (const auto& cmd : commands_) {
        if (cmd.longName == command || cmd.shortName == command) {

            cmd.funcPtr(arguments, response);
            return;
        }
    }

    // Print the name of the command that was not found, and suggest using "help" to see available commands.
    uart_->println("Command not found: " + String(command.c_str()));
    uart_->println("Type 'help' or '?' to see available commands.");
}

// Help function to list all commands with their long and short names
void CommandLine::help(){
    if (commands_.empty()) {
        uart_->println("No commands available.");
        uart_->println("help<?>");
        return;
    }
 
    for (const auto& cmd : commands_) {
        uart_->println(String(cmd.longName.c_str()) + "<" + String(cmd.shortName.c_str()) + ">");  
    }
    uart_->println("help<?>");

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
