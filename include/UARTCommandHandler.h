#ifndef UARTCOMMANDHANDLER_H
#define UARTCOMMANDHANDLER_H

#include <functional>
#include <queue>
#include <string>
#include <vector>

#include "ArduinoHAL.h"

enum class AsciiKey : uint8_t {
    Escape      = 27,
    UpArrow     = 65,
    DownArrow   = 66,
    Backspace   = 8,
    Delete      = 127
};

constexpr int UART_BUFFER_SIZE = 128;
constexpr int MAX_HISTORY = 20;
constexpr int MAX_ARGUMENTS = 5;
constexpr int MAX_ROW_LENGTH = 40;

/**
 * @brief Lightweight UART command-line interface with history and parsing.
 * @note When to use: add interactive commands for debugging or configuration
 *       over a serial terminal without bringing in a full shell.
 */
class CommandLine {

public:
    CommandLine(Stream * UART);  // Constructor
    void addCommand(const std::string& longName, const std::string& shortName, std::function<void(std::queue<std::string> argumentQueue ,std::string&)> funcPtr);
    void executeCommand(const std::string& command, std::queue<std::string> arugments);
    void readInput();
    void processCommand(const std::string& command);
    void begin();

    // Pass-through functions for the UART object
    void println(const std::string& message){
        UART->println(message.c_str());
    }
    void print(const std::string& message){
        UART->print(message.c_str());
    }

private:
    Stream * UART;  // Pointer to the UART object
    struct Command {
        std::string longName;             
        std::string shortName;           
        std::function<void(std::queue<std::string>, std::string&)> funcPtr; // Function pointer to the command handler
    };
    std::vector<Command> commands{};  // Vector to store the list of commands

    // Class member variables for buffering and history
    std::string fullLine = {""}; 
    std::string inputBuffer = "";  // Current input buffer
    std::string argBuffer = "";
    std::string command = "";
    int historyIndex = 0;  // Keeps track of the current position in the command history
    std::queue<std::string> argumentQueue{};
    bool isCommandParsed = false;

    std::array<std::string, MAX_HISTORY> fullLineHistory = {};
    std::array<std::string, MAX_HISTORY> commandHistory = {};
    std::array<std::array<std::string, MAX_ARGUMENTS>, MAX_HISTORY> argumentHistory = {};
    std::array<int, MAX_HISTORY> argSize = {};


    void help();
    void displayCommandFromHistory();
    std::string combineArguments(std::queue<std::string> arguments);
    void trimSpaces(std::string& str);
};

#endif