#ifndef COBRA_LINE_SENSOR_HPP
#define COBRA_LINE_SENSOR_HPP

#include <Arduino.h>
#include "pin_map.hpp"
#include "driver/adc.h"
#include "esp_adc_cal.h"

// ADC Callibration
#define DEFAULT_VREF 1100 // Default reference voltage in mV
// TODO:  Use adc2_vref_to_gpio() to obtain a better estimate

// ADC Characterization struct
esp_adc_cal_characteristics_t *adc_chars;

/**
 * @brief Setup the Cobra Line Sensor
 *
 * This function configures the ADC width and attenuation for the line sensors.
 * It also characterizes the ADC at the defaul reference voltage and 12-bit resolution.
 */
void setupCobraSensor()
{
    // Configure ADC width and attenuation
    adc1_config_width(ADC_WIDTH_BIT_12);                 // 12 bit resolution
    adc1_config_channel_atten(CobraIR1, ADC_ATTEN_11db); // 0-3.6V range
    adc1_config_channel_atten(CobraIR2, ADC_ATTEN_11db); // 0-3.6V range
    adc1_config_channel_atten(CobraIR3, ADC_ATTEN_11db); // 0-3.6V range
    adc1_config_channel_atten(CobraIR4, ADC_ATTEN_11db); // 0-3.6V range

    // Characterize ADC at default Vref and 12 bit resolution
    adc_chars = (esp_adc_cal_characteristics_t *)calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_11db, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);

    // Serial.println("Setup complete. Reading line sensors...");
}

/**
 * @brief Read raw ADC Value from the specified line sensor channel
 *
 * @param channel ADC channel to read from
 * @return int32_t Raw ADC value
 */
int32_t readLineSensorRaw(adc1_channel_t channel)
{
    // Read ADC value
    int32_t adc_reading = adc1_get_raw(channel);

    return adc_reading;
}

/**
 * @brief Read voltage from the specified line sensor channel.
 *
 * This function reads the raw ADC value and converts it to voltage in millivolts.
 *
 * @param channel The ADC channel to read from.
 * @return int32_t The voltage in millivolts.
 */
int32_t readLineSensorVoltage(adc1_channel_t channel)
{
    // Read ADC value
    int32_t adc_reading = adc1_get_raw(channel);

    // Convert ADC reading to voltage in mV
    uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);

    return voltage;
}

#endif // COBRA_LINE_SENSOR_HPP