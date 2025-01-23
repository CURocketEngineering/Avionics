#ifndef UARTCOMMANDHANDLER_H
#define UARTCOMMANDHANDLER_H

#include <iostream>
#include <string>
#include <vector>
#include <functional>


#define BUFFER_SIZE 128

class CommandLine {

public:
    CommandLine();  // Constructor
    void addCommand(const std::string& longName, const std::string& shortName, std::function<void(const std::string&, const std::string&)> funcPtr);
    void executeCommand(const std::string& command);
    void readInput();
    void processCommand(const std::string& command);

private:
    struct Command {
        std::string longName;             // The command string
        std::string shortName;            // The response string
        std::function<void(const std::string&, const std::string&)> funcPtr; // Function pointer to the command handler
    };
    std::vector<Command> commands;  // Vector to store the list of commands

    // Class member variables for buffering and history
    std::string inputBuffer = "";  // Current input buffer
    std::vector<std::string> commandHistory;  // Stores previous commands for up-arrow navigation
    int historyIndex = -1;  // Keeps track of the current position in the command history


    void help();
    void displayCommandFromHistory();
};

#endif