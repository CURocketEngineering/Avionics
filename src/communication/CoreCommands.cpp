#include "communication/CoreCommands.h"
#include "communication/CommandHandler.h"
#include <functional>

CoreCommands::CoreCommands() {
}

std::vector<uint8_t> CoreCommands::handlePing(const std::vector<uint8_t>& data) {
    // Simple ping response: OK status
    return std::vector<uint8_t>{RESP_OK};
}

std::vector<uint8_t> CoreCommands::handleVersions(const std::vector<uint8_t>& data) {
    std::vector<uint8_t> response = createResponse(RESP_OK);
    
    // Add number of versions
    response.push_back(static_cast<uint8_t>(versions.size()));
    
    // Add each version entry
    for (const auto& ver : versions) {
        appendString(response, ver.component);
        appendString(response, ver.version);
    }
    
    return response;
}

void CoreCommands::addVersion(const std::string& component, const std::string& version) {
    versions.emplace_back(component, version);
}

void CoreCommands::clearVersions() {
    versions.clear();
}

void CoreCommands::bindToHandler(std::shared_ptr<CommandHandler> handler) {
    // Create a shared pointer to CoreCommands
    auto core = std::make_shared<CoreCommands>();
    
    // Bind core command handlers
    handler->bindCommand(getPingCommand(), 
        std::bind(&CoreCommands::handlePing, core, std::placeholders::_1));
    
    handler->bindCommand(getVersionsCommand(),
        std::bind(&CoreCommands::handleVersions, core, std::placeholders::_1));
}

std::vector<uint8_t> CoreCommands::createResponse(uint8_t status) {
    return std::vector<uint8_t>{status};
}

std::vector<uint8_t> CoreCommands::createResponse(uint8_t status, const std::string& data) {
    std::vector<uint8_t> response{status};
    appendString(response, data);
    return response;
}

void CoreCommands::appendString(std::vector<uint8_t>& response, const std::string& str) {
    // Add string length
    response.push_back(static_cast<uint8_t>(str.length()));
    
    // Add string data
    response.insert(response.end(), str.begin(), str.end());
}