#ifndef COMMAND_MANAGER_H
#define COMMAND_MANAGER_H

#include <map>
#include <functional>
#include <Arduino.h>
#include <TelnetStream.h>

class CommandManager
{
public:
    // Constructor
    CommandManager();

    // Add a command and its associated function
    void add_command(const String &command, std::function<void()> func);

    // Execute the function associated with a command
    void execute_command(const String &command);

    // Set the stop flag
    void stop_execution();

    // Check if execution should stop
    bool should_stop() const;

private:
    std::map<String, std::function<void()>> commandMap; // Command-function map
    bool stopFlag;                                      // Flag to stop long-running commands
};

#endif