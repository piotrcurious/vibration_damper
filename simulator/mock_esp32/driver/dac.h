#ifndef DAC_H
#define DAC_H

#include <cstdint>

typedef enum {
    DAC_CHANNEL_1 = 1,    /*!< DAC channel 1 is GPIO25 */
    DAC_CHANNEL_2 = 2,    /*!< DAC channel 2 is GPIO26 */
} dac_channel_t;

void dac_output_enable(dac_channel_t channel);
void dac_output_voltage(dac_channel_t channel, uint8_t voltage);

#endif
