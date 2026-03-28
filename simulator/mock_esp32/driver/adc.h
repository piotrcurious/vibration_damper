#ifndef ADC_H
#define ADC_H

#include <cstdint>

typedef enum {
    ADC1_CHANNEL_0 = 0,
    ADC1_CHANNEL_1,
    ADC1_CHANNEL_2,
    ADC1_CHANNEL_3,
    ADC1_CHANNEL_4,
    ADC1_CHANNEL_5,
    ADC1_CHANNEL_6,
    ADC1_CHANNEL_7,
    ADC1_CHANNEL_MAX,
} adc1_channel_t;

typedef enum {
    ADC_WIDTH_BIT_12 = 3,
} adc_bits_width_t;

typedef enum {
    ADC_ATTEN_DB_0 = 0,
    ADC_ATTEN_DB_2_5 = 1,
    ADC_ATTEN_DB_6 = 2,
    ADC_ATTEN_DB_11 = 3,
} adc_atten_t;

void adc1_config_width(adc_bits_width_t width_bit);
void adc1_config_channel_atten(adc1_channel_t channel, adc_atten_t atten);
int adc1_get_raw(adc1_channel_t channel);

#endif
