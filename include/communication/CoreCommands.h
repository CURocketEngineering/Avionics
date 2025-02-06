#ifndef CORE_COMMANDS_H
#define CORE_COMMANDS_H

#include <string>
#include <vector>
#include <memory>

// Version information structure
struct VersionInfo {
    std::string component;
    std::string version;
    
    VersionInfo(const std::string& comp, const std::string& ver)
        : component(comp), version(ver) {}
};

class CoreCommands {
public:
    // Response codes
    static const uint8_t RESP_OK = 0x00;
    static const uint8_t RESP_ERROR = 0xFF;

    CoreCommands();
    
    // Core command handlers that return response bytes
    std::vector<uint8_t> handlePing(const std::vector<uint8_t>& data);
    std::vector<uint8_t> handleVersions(const std::vector<uint8_t>& data);

    // Version management
    void addVersion(const std::string& component, const std::string& version);
    void clearVersions();
    
    // Static command code getters
    static std::string getPingCommand() { return "\x01"; }
    static std::string getVersionsCommand() { return "\x02"; }
    
    // Factory method to create and bind to a command handler
    static void bindToHandler(std::shared_ptr<class CommandHandler> handler);

private:
    std::vector<VersionInfo> versions;
    
    // Helper methods for response formatting
    static std::vector<uint8_t> createResponse(uint8_t status);
    static std::vector<uint8_t> createResponse(uint8_t status, const std::string& data);
    static void appendString(std::vector<uint8_t>& response, const std::string& str);
};

#endif // CORE_COMMANDS_H