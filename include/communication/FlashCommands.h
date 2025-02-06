#ifndef FLASH_COMMANDS_H
#define FLASH_COMMANDS_H

#include <vector>
#include <string>
#include <cstdint>
#include "FlashDriver.h"

class FlashCommands {
public:
    // Response codes
    static const uint8_t RESP_OK = 0x00;
    static const uint8_t RESP_ERROR = 0xFF;

    FlashCommands(FlashDriver* flash);

    // Flash command handlers
    std::vector<uint8_t> handleFlashDump(const std::vector<uint8_t>& data);

    // Command code getters
    static std::string getFlashDumpCommand() { return "\x03"; }

private:
    FlashDriver* flashDriver;

    // Helper methods
    static std::vector<uint8_t> createResponse(uint8_t status);
    static std::vector<uint8_t> createResponse(uint8_t status, const std::vector<uint8_t>& data);
};

#endif // FLASH_COMMANDS_H