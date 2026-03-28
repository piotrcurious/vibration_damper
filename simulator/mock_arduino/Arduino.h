#ifndef ARDUINO_H
#define ARDUINO_H

#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <cstring>
#include <chrono>
#include <thread>
#include <algorithm>

#define IRAM_ATTR
#define PI 3.14159265358979323846

class String : public std::string {
public:
    using std::string::string;
    String(const std::string& s) : std::string(s) {}
    void trim() {
        this->erase(0, this->find_first_not_of(' '));
        this->erase(this->find_last_not_of(' ') + 1);
    }
    void toUpperCase() {
        std::transform(this->begin(), this->end(), this->begin(), ::toupper);
    }
    bool startsWith(const char* s) const {
        return this->find(s) == 0;
    }
    String substring(int from) const {
        return String(this->substr(from));
    }
    float toFloat() const {
        try {
            return std::stof(*this);
        } catch (...) {
            return 0.0f;
        }
    }
};

class SerialMock {
public:
    void begin(int baud) {}
    void print(const String& s) { std::cout << s; }
    void print(const char* s) { std::cout << s; }
    void print(double d) { std::cout << d; }
    void println(const String& s) { std::cout << s << std::endl; }
    void println(const char* s) { std::cout << s << std::endl; }
    void println(double d) { std::cout << d << std::endl; }
    void println() { std::cout << std::endl; }
    template<typename... Args>
    void printf(const char* format, Args... args) {
        char buf[256];
        snprintf(buf, sizeof(buf), format, args...);
        std::cout << buf;
    }
    bool available() { return false; }
    String readStringUntil(char terminator) { return ""; }
};

extern SerialMock Serial;

// Simulator functions to call during delay
void update_plant_sim();

inline void delay(int ms) {
    for (int i=0; i<ms; ++i) {
        for (int j=0; j<1000/250; ++j) { // Assume 4kHz, so 4 ticks per ms
            update_plant_sim();
        }
    }
}

inline void delayMicroseconds(int us) {
    if (us >= 250) { // SAMPLE_PERIOD_US is 250
        update_plant_sim();
    }
}

// Global time in milliseconds, updated by plant sim
extern float t_global;
inline uint32_t millis() {
    return (uint32_t)(t_global * 1000.0f);
}

inline uint32_t esp_random() {
    return rand();
}

#define pdMS_TO_TICKS(ms) (ms)

#endif
