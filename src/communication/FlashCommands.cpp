#include "communication/FlashCommands.h"

FlashCommands::FlashCommands(FlashDriver* flash) 
    : flashDriver(flash) {
}

std::vector<uint8_t> FlashCommands::handleFlashDump(const std::vector<uint8_t>& data) {
    if (!flashDriver) {
        return createResponse(RESP_ERROR);
    }

    // Read the entire flash contents
    std::vector<uint8_t> flashData;
    size_t flashSize = flashDriver->size();
    flashData.reserve(flashSize);

    for (size_t addr = 0; addr < flashSize; addr++) {
        uint8_t byte;
        if (!flashDriver->readByte(addr, byte)) {
            return createResponse(RESP_ERROR);
        }
        flashData.push_back(byte);
    }

    return createResponse(RESP_OK, flashData);
}

std::vector<uint8_t> FlashCommands::createResponse(uint8_t status) {
    return std::vector<uint8_t>{status};
}

std::vector<uint8_t> FlashCommands::createResponse(uint8_t status, const std::vector<uint8_t>& data) {
    std::vector<uint8_t> response;
    response.reserve(1 + 4 + data.size()); // status + size + data

    // Add status byte
    response.push_back(status);

    // Add data length (4 bytes, little-endian)
    uint32_t length = data.size();
    response.push_back(length & 0xFF);
    response.push_back((length >> 8) & 0xFF);
    response.push_back((length >> 16) & 0xFF);
    response.push_back((length >> 24) & 0xFF);

    // Add data
    response.insert(response.end(), data.begin(), data.end());

    return response;
}