#ifndef UARTCOMMANDHANDLER_H
#define UARTCOMMANDHANDLER_H

#include <string>
#include <queue>
#include <vector>
#include <functional>
#include <HardwareSerial.h>

#define UART_BUFFER_SIZE 128
extern HardwareSerial UART;

using namespace std;

class CommandLine {

public:
    CommandLine();  // Constructor
    void addCommand(const string& longName, const string& shortName, function<void(queue<string> argumentQueue , string&)> funcPtr);
    void executeCommand(const string& command, queue<string> arugments);
    void readInput();
    void processCommand(const string& command);
    void begin();

private:
    struct Command {
        string longName;             
        string shortName;           
        function<void(queue<string>, string&)> funcPtr; // Function pointer to the command handler
    };
    vector<Command> commands;  // Vector to store the list of commands

    // Class member variables for buffering and history
    string inputBuffer = "";  // Current input buffer
    string argBuffer = "";
    string command = "";
    vector<string> commandHistory;  // Stores previous commands for up-arrow navigation
    int historyIndex = -1;  // Keeps track of the current position in the command history
    queue<string> argumentQueue;
    bool isCommandParsed = false;
    bool newComandLine = false;


    void help();
    void displayCommandFromHistory();
    string combineArguments(queue<string> arguments);
    void trimSpaces(std::string& str);
};

#endif