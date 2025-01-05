#ifndef SHARPIR_DRIVER_HPP
#define SHARPIR_DRIVER_HPP

#include <Arduino.h>
#include "pin_map.hpp"
#include "driver/adc.h"
#include "esp_adc_cal.h"

// ADC Callibration
#define DEFAULT_VREF 1100 // Default reference voltage in mV

// ADC Characterization struct
esp_adc_cal_characteristics_t *adc_chars;

/**
 * @brief Setup the Sharp IR Sensor
 *
 * This function configures the ADC width and attenuation for the Sharp IR Sensors.
 * It also characterizes the ADC at the defaul reference voltage and 12-bit resolution.
 */
void setupSharpIRsensor()
{
    adc2_config_channel_atten(SharpIR1, ADC_ATTEN_11db); // 0-3.6V range
    adc2_config_channel_atten(SharpIR2, ADC_ATTEN_11db); // 0-3.6V range

    // Characterize ADC at default Vref and 12 bit resolution
    adc_chars = (esp_adc_cal_characteristics_t *)calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_characterize(ADC_UNIT_2, ADC_ATTEN_11db, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);

}

/**
 * @brief Read raw ADC Value from the specified SharpIR Sensor
 * @param channel ADC channel to read from
 * @return int32_t Raw ADC value
 */
int32_t readIRSensorRaw(adc2_channel_t channel)
{
    int raw_value = 0;
    esp_err_t r = adc2_get_raw(channel, ADC_WIDTH_BIT_12, &raw_value);
    if (r == ESP_OK)
    {
        return raw_value;
    }

    return -1; // Error reading ADC
}

/**
 * @brief Read voltage from the specified SharpIR sensor channel.
 *
 * This function reads the raw ADC value and converts it to voltage in millivolts.
 *
 * @param channel The ADC channel to read from.
 * @return int32_t The voltage in millivolts.
 */
int32_t readIRSensorVoltage(adc2_channel_t channel)
{
    int raw_value;
    esp_err_t r = adc2_get_raw(channel, ADC_WIDTH_BIT_12, &raw_value);
    if (r == ESP_OK) {
        return esp_adc_cal_raw_to_voltage(raw_value, adc_chars);
    }
    return -1; // Error reading ADC
}

float getDistanceinCM(adc2_channel_t channel)
{
    int raw_value;
    esp_err_t r = adc2_get_raw(channel, ADC_WIDTH_BIT_12, &raw_value);
    if (r == ESP_OK) {
        float volts = raw_value * 0.0008056640625; // value from sensor * (3.3/4096)
        return 29.988 * pow(volts, -1.173);
    }
    return -1; // Error reading ADC
}

#endif // SHARPIR_DRIVER_HPP
