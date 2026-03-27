#include "unity_config.h"

#include <cstdio>

#if !defined(UNITY_WEAK_ATTRIBUTE) && !defined(UNITY_WEAK_PRAGMA)
#if defined(__GNUC__) || defined(__ghs__)
#if !(defined(__WIN32__) && defined(__clang__)) && !defined(__TMS470__)
#define UNITY_WEAK_ATTRIBUTE __attribute__((weak))
#endif
#endif
#endif

extern "C" {

#ifdef UNITY_WEAK_ATTRIBUTE
UNITY_WEAK_ATTRIBUTE void setUp(void) {}
UNITY_WEAK_ATTRIBUTE void tearDown(void) {}
UNITY_WEAK_ATTRIBUTE void suiteSetUp(void) {}
UNITY_WEAK_ATTRIBUTE int suiteTearDown(int num_failures) { return num_failures; }
#elif defined(UNITY_WEAK_PRAGMA)
#pragma weak setUp
void setUp(void) {}
#pragma weak tearDown
void tearDown(void) {}
#pragma weak suiteSetUp
void suiteSetUp(void) {}
#pragma weak suiteTearDown
int suiteTearDown(int num_failures) { return num_failures; }
#endif

void unityOutputStart(unsigned long baudrate) {
    (void)baudrate;
}

void unityOutputChar(unsigned int c) {
    std::putchar(static_cast<int>(static_cast<unsigned char>(c)));
}

void unityOutputFlush(void) {
    std::fflush(stdout);
}

void unityOutputComplete(void) {}

}
