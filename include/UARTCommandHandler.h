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

using namespace std;

class CommandLine {

public:
    CommandLine(Stream * UART);  // Constructor
    void addCommand(const string& longName, const string& shortName, function<void(queue<string> argumentQueue , string&)> funcPtr);
    void executeCommand(const string& command, queue<string> arugments);
    void readInput();
    void processCommand(const string& command);
    void begin();

    // Pass-through functions for the UART object
    void println(const string& message){
        UART->println(message.c_str());
    }
    void print(const string& message){
        UART->print(message.c_str());
    }

private:
    Stream * UART;  // Pointer to the UART object
    struct Command {
        string longName;             
        string shortName;           
        function<void(queue<string>, string&)> funcPtr; // Function pointer to the command handler
    };
    vector<Command> commands{};  // Vector to store the list of commands

    // Class member variables for buffering and history
    string fullLine = {""}; 
    string inputBuffer = "";  // Current input buffer
    string argBuffer = "";
    string command = "";
    int historyIndex = 0;  // Keeps track of the current position in the command history
    queue<string> argumentQueue{};
    bool isCommandParsed = false;

    std::array<std::string, MAX_HISTORY> fullLineHistory = {};
    std::array<std::string, MAX_HISTORY> commandHistory = {};
    std::array<std::array<std::string, MAX_ARGUMENTS>, MAX_HISTORY> argumentHistory = {};
    std::array<int, MAX_HISTORY> argSize = {};


    void help();
    void displayCommandFromHistory();
    string combineArguments(queue<string> arguments);
    void trimSpaces(std::string& str);
};

#endif