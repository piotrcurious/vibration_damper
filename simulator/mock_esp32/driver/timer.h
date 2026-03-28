#ifndef TIMER_H
#define TIMER_H

#include <cstdint>

typedef void* hw_timer_t;

inline hw_timer_t* timerBegin(uint8_t timer, uint16_t divider, bool countUp) { return nullptr; }
inline void timerAttachInterrupt(hw_timer_t* timer, void (*fn)(), bool edge) {}
inline void timerAlarmWrite(hw_timer_t* timer, uint64_t alarm_value, bool autoreload) {}
inline void timerAlarmEnable(hw_timer_t* timer) {}

#endif
