#ifndef SPI_MOCK_H
#define SPI_MOCK_H

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>

/* ──────────────────────────────
   Simple Arduino‑style helpers
   ──────────────────────────────*/
#define F(x) x                 // PROGMEM macro stub

class MockSerialClass {
public:
    template<typename T> void print(const T&)   {}
    template<typename T> void println(const T&) {}
    // clear mock
    void clear() {
    }
};
// inline MockSerialClass Serial;  // global mock instance

/* ──────────────────────────────
   SPI mock
   ──────────────────────────────*/
#define MSBFIRST  0
#define SPI_MODE0 0

class SPISettings {
public:
    SPISettings(int, int, int) {}
};

class MockSPIClass {
public:
    MockSPIClass(uint32_t&, uint32_t&, uint32_t&, uint32_t&) {}
    MockSPIClass() {}

    static void begin()                              {}
    static void end()                                {}
    static void beginTransaction(SPISettings)        {}
    static void endTransaction()                     {}

    /* dumb stubs that always “succeed” */
    static bool transfer(uint8_t&)                           { return true; }
    static bool transfer(uint8_t*, std::nullptr_t, int)      { return true; }
    static bool transfer(uint8_t*, uint8_t*&, size_t&)       { return true; }
    static bool transfer(const uint8_t*, std::nullptr_t, int){ return true; }
    static bool transfer(uint8_t*, uint8_t, int)             { return true; }
    static bool transfer(uint8_t*, uint8_t*, int)            { return true; }
    static bool transfer(uint8_t*, uint8_t*, int*)           { return true; }

    static bool writeBytes(const uint8_t*, int)              { return true; }
    static bool writeBytes(uint8_t*, uint8_t*, int)          { return true; }
    static bool writeBytes(uint8_t*, uint8_t*, int*)         { return true; }

    static bool transferBytes(uint8_t*, uint8_t*, int)       { return true; }
    static bool transferBytes(uint8_t*, uint8_t*, int*)      { return true; }
};

class SPIClass : public MockSPIClass {
public:
    SPIClass(uint32_t& mosi, uint32_t& miso,
             uint32_t& sclk, uint32_t& ssel)
        : MockSPIClass(mosi, miso, sclk, ssel) {}
    SPIClass() : MockSPIClass() {}
};
extern SPIClass SPI;           // declared; you can define in a test .cpp

/* ──────────────────────────────
   SdFat / File mocks
   ──────────────────────────────*/
struct File32 {
    bool   open(const char*, int)          { return true; }
    void   close()                         {}
    bool   exists(const char*)             { return false; }

    /* Arduino‑style file API */
    int    write(const void*, size_t n)    { return static_cast<int>(n); }
    void   sync()                          {}

    template<typename T> void print  (const T&) {}
    template<typename T> void println(const T&) {}

    bool   preAllocate(uint32_t)           { return true; }

    /* truthiness test:  if(file) … */
    explicit operator bool() const         { return true; }
};
using SdFile_t = File32;

struct SdFat {
    bool begin(uint8_t, int)               { return true; }
    bool exists(const char*)               { return false; }
    SdFile_t open(const char*, int)        { return SdFile_t(); }
};

/* file‑open flags + helper macro */
#define O_WRITE  0x01
#define O_CREAT  0x02
#define O_APPEND 0x04
#define SD_SCK_MHZ(x) (x)

#endif /* SPI_MOCK_H */
