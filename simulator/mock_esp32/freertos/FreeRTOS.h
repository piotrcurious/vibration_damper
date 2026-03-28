#ifndef FREERTOS_H
#define FREERTOS_H

#include <cstdint>

typedef void* TaskHandle_t;
typedef void (*TaskFunction_t)(void*);

#define portMUX_TYPE int
#define portMUX_INITIALIZER_UNLOCKED 0
#define portENTER_CRITICAL_ISR(mux)
#define portEXIT_CRITICAL_ISR(mux)
#define portENTER_CRITICAL(mux)
#define portEXIT_CRITICAL(mux)

inline void vTaskDelay(uint32_t ticks) {
    // In mock, we don't block. Tasks are called at regular intervals by main.
}

void xTaskCreatePinnedToCore(TaskFunction_t task, const char* name, uint32_t stack, void* param, int prio, TaskHandle_t* handle, int core);

#endif
