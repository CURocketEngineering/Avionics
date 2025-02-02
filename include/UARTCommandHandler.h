#ifndef UARTCOMMANDHANDLER_H
#define UARTCOMMANDHANDLER_H

#include <string>
#include <queue>
#include <vector>
#include <functional>
#include "ArduinoHAL.h"

#define UART_BUFFER_SIZE 128
#define MAX_HISTORY      20
#define MAX_ARGUMENTS    5

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
    vector<Command> commands;  // Vector to store the list of commands

    // Class member variables for buffering and history
    string fullLine = {""}; 
    string inputBuffer = "";  // Current input buffer
    string argBuffer = "";
    string command = "";
    string fullLineHistory[MAX_HISTORY] = {""}; 
    string commandHistory[MAX_HISTORY];  // Stores previous commands for up-arrow navigation
    int historyIndex = 0;  // Keeps track of the current position in the command history
    queue<string> argumentQueue;
    string argumentHistory[MAX_HISTORY][MAX_ARGUMENTS];
    int argSize[MAX_HISTORY];
    bool isCommandParsed = false;


    void help();
    void displayCommandFromHistory();
    string combineArguments(queue<string> arguments);
    void trimSpaces(std::string& str);
};

#endif