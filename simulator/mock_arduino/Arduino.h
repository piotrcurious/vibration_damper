#ifndef MOCK_ARDUINO_H
#define MOCK_ARDUINO_H

#include <iostream>
#include <cstdint>
#include <cstring>
#include <cmath>
#include <string>
#include <algorithm>

#ifndef PI
#define PI 3.14159265358979323846f
#endif

typedef void (*voidFuncPtr)(void);
void set_delay_callback(voidFuncPtr cb);

void delay(uint32_t ms);
void delayMicroseconds(uint32_t us);
uint32_t millis();
uint32_t micros();

class String : public std::string {
public:
    String(const char* s = "") : std::string(s) {}
    String(const std::string& s) : std::string(s) {}

    void trim() {
        erase(0, find_first_not_of(" \t\r\n"));
        size_t last = find_last_not_of(" \t\r\n");
        if (last != std::string::npos) erase(last + 1);
    }

    void toUpperCase() {
        std::transform(begin(), end(), begin(), ::toupper);
    }

    bool startsWith(const char* s) const {
        return find(s) == 0;
    }

    String substring(int start) const {
        return String(this->substr(start));
    }

    float toFloat() const {
        try {
            return std::stof(*this);
        } catch (...) {
            return 0.0f;
        }
    }

    int toInt() const {
        try {
            return std::stoi(*this);
        } catch (...) {
            return 0;
        }
    }
};

class SerialMock {
public:
    void begin(int baud) {}
    void print(const char* s) { std::cout << s; }
    void print(float f) { std::cout << f; }
    void print(int i) { std::cout << i; }
    void println(const char* s) { std::cout << s << std::endl; }
    void println(float f) { std::cout << f << std::endl; }
    void println(int i) { std::cout << i << std::endl; }
    void printf(const char* fmt, ...);
    int available() { return 0; }
    String readStringUntil(char terminator) { return String(""); }
};

extern SerialMock Serial;

uint32_t esp_random();

#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif

#ifndef portMUX_TYPE
#define portMUX_TYPE int
#define portMUX_INITIALIZER_UNLOCKED 0
#endif

#ifndef portENTER_CRITICAL_ISR
#define portENTER_CRITICAL_ISR(x)
#define portEXIT_CRITICAL_ISR(x)
#define portENTER_CRITICAL(x)
#define portEXIT_CRITICAL(x)
#endif

#define pdMS_TO_TICKS(x) (x)

#endif
