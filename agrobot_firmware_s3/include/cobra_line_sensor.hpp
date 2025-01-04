#ifndef COBRA_LINE_SENSOR_HPP
#define COBRA_LINE_SENSOR_HPP

#include <Arduino.h>
#include "pin_map.hpp"
#include "driver/adc.h"
#include "esp_adc_cal.h"

#define LineSensorCount 4

// ADC Calibration
#define DEFAULT_VREF 1100 // Default reference voltage in mV
// TODO:  Use adc2_vref_to_gpio() to obtain a better estimate

/**
 * @brief Class to handle Cobra Line Sensor operations.
 *
 * This class provides methods to setup the Cobra Line Sensor, update readings,
 * and get readings in both raw and voltage forms.
 */
class CobraLineSensor
{
public:
    /**
     * @brief Constructor for CobraLineSensor.
     *
     * Initializes the ADC characterization structure.
     */
    CobraLineSensor();

    /**
     * @brief Setup the Cobra Line Sensor.
     *
     * This function configures the ADC width and attenuation for the line sensors.
     * It also characterizes the ADC at the default reference voltage and 12-bit resolution.
     */
    void setup();

    /**
     * @brief Update the raw readings from the line sensors.
     *
     * This function reads the raw ADC values from all line sensors and stores them.
     */
    void updateRawReadings();

    /**
     * @brief Update the voltage readings from the line sensors.
     *
     * This function reads the voltage values from all line sensors and stores them.
     */
    void updateVoltageReadings();

    /**
     * @brief Get the voltage reading from a specific line sensor.
     *
     * @param index The index of the line sensor (0 to 3).
     * @return int32_t The voltage reading in millivolts.
     */
    int32_t getVoltageReading(int index) const;

    /**
     * @brief Get the raw reading from a specific line sensor.
     *
     * @param index The index of the line sensor (0 to 3).
     * @return int32_t The raw ADC reading.
     */
    int32_t getRawReading(int index) const;

private:
    esp_adc_cal_characteristics_t *adc_chars; ///< ADC characterization structure
    int32_t voltageReadings[LineSensorCount]; ///< Array to store voltage readings
    int32_t rawReadings[LineSensorCount];     ///< Array to store raw ADC readings
};

CobraLineSensor::CobraLineSensor()
{
    adc_chars = (esp_adc_cal_characteristics_t *)calloc(1, sizeof(esp_adc_cal_characteristics_t));
}

void CobraLineSensor::setup()
{
    // Configure ADC width and attenuation
    adc1_config_width(ADC_WIDTH_BIT_12);                 // 12 bit resolution
    adc1_config_channel_atten(CobraIR1, ADC_ATTEN_11db); // 0-3.6V range
    adc1_config_channel_atten(CobraIR2, ADC_ATTEN_11db); // 0-3.6V range
    adc1_config_channel_atten(CobraIR3, ADC_ATTEN_11db); // 0-3.6V range
    adc1_config_channel_atten(CobraIR4, ADC_ATTEN_11db); // 0-3.6V range

    // Characterize ADC at default Vref and 12 bit resolution
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_11db, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
}

void CobraLineSensor::updateRawReadings()
{
    rawReadings[0] = adc1_get_raw(CobraIR1);
    rawReadings[1] = adc1_get_raw(CobraIR2);
    rawReadings[2] = adc1_get_raw(CobraIR3);
    rawReadings[3] = adc1_get_raw(CobraIR4);
}

void CobraLineSensor::updateVoltageReadings()
{
    voltageReadings[0] = esp_adc_cal_raw_to_voltage(adc1_get_raw(CobraIR1), adc_chars);
    voltageReadings[1] = esp_adc_cal_raw_to_voltage(adc1_get_raw(CobraIR2), adc_chars);
    voltageReadings[2] = esp_adc_cal_raw_to_voltage(adc1_get_raw(CobraIR3), adc_chars);
    voltageReadings[3] = esp_adc_cal_raw_to_voltage(adc1_get_raw(CobraIR4), adc_chars);
}

int32_t CobraLineSensor::getVoltageReading(int index) const
{
    if (index >= 0 && index < LineSensorCount)
    {
        return voltageReadings[index];
    }
    return -1; // Return an invalid value if index is out of range
}

int32_t CobraLineSensor::getRawReading(int index) const
{
    if (index >= 0 && index < LineSensorCount)
    {
        return rawReadings[index];
    }
    return -1; // Return an invalid value if index is out of range
}

#endif // COBRA_LINE_SENSOR_HPP