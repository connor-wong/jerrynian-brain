#include <command_manager.h>

CommandManager::CommandManager() : stopFlag(false) {}

void CommandManager::add_command(const String &command, std::function<void()> func)
{
    commandMap[command] = func;
}

void CommandManager::execute_command(const String &command)
{
    if (commandMap.find(command) != commandMap.end())
    {
        stopFlag = false;      // Reset stop flag before executing a command
        commandMap[command](); // Call the associated function
    }
    else
    {
        TelnetStream.println("Error: Command not found - " + command);
    }
}

void CommandManager::stop_execution()
{
    stopFlag = true;
}

bool CommandManager::should_stop() const
{
    return stopFlag;
}