/*------------------------------------------------------------------------------
--                   HERCCULES On-Board Software Components                   --
--                                                                            --
--                           EQUIPMENT  HANDLERS                              --
--                                                                            --
--                            BNO055_IMU Header                               --
--                                                                            --
--            Copyright (C) 2022 Universidad Politécnica de Madrid            --
--                                                                            --
-- HERCCULES was developed by the Real-Time Systems Group at  the Universidad --
-- Politécnica de Madrid.                                                     --
--                                                                            --
-- HERCCULES is free software: you can redistribute it and/or modify it under --
-- the terms of the GNU General Public License as published by the Free Soft- --
-- ware Foundation, either version 3 of the License,  or (at your option) any --
-- later version.                                                             --
--                                                                            --
-- HERCCULES is distributed  in the hope that  it will be useful  but WITHOUT --
-- ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FIT- --
-- NESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more --
-- details. You should have received a copy of the GNU General Public License --
-- along with HERCCULES. If not, see <https://www.gnu.org/licenses/>.         --
--                                                                            --
------------------------------------------------------------------------------*/

// The implementation for this handler is based on the device driver interface
// written for the Ada Drivers Library, available at:
// https://github.com/AdaCore/Ada_Drivers_Library/blob/master/components/src/motion/bno055/bosch_bno055.ads

#ifndef HAL_BNO055_IMU_H
#define HAL_BNO055_IMU_H

#include "BusHandlers_Data.h"

#include <cstdint>

namespace equipment_handlers {

    /**
     * This class provides an interface for the Bosch 9-DOF Absolute
     * Orientation IMU, integrating a triaxial 14-bit accelerometer,
     * a triaxial geomagnetic sensor, and 32 bit cortex M0+ uController
     * running Bosch Sensortec sensor function software,
     * in a single package (SiP).
     *
     * The device supports I2C and UART interfaces. This class is
     * configured to work with an I2C IF.
     */
    class BNO055_IMU {
    public:


        /// These are the two possible I2C addresses for the BNO055:
        /// @{
        static const uint8_t BNO055_Primary_Address   = 0x28U;
        static const uint8_t BNO055_Alternate_Address = 0x29U;
        /// @}

        enum PowerModes : uint8_t {
            NORMAL   = 0x00U,
            // All the required sensors are switched on.
            LOWPOWER = 0x01U,
            // No motion for T seconds -> only accel is active,
            // Motion for T seconds -> all sensors are active.
            SUSPEND  = 0x02U
            // All sensors are in sleep mode.
        };

        enum OperatingModes : uint8_t {
            CONFIG = 0x00U,
            ACC_ONLY = 0x01U,
            MAG_ONLY = 0x02U,
            GYRO_ONLY = 0x03U,
            ACC_MAG = 0x04U,
            ACC_GYRO = 0x05U,
            MAG_GYRO = 0x06U,
            AMG = 0x07U,
            IMU = 0x08U,
            COMPASS = 0x09U,
            M4G = 0x0AU,
            NDOF_FMS_OFF = 0x0BU,
            NDOF = 0x0CU
        };

        // --------------------------------
        // -- Initialization & reset ops --
        // --------------------------------

        bool initialize
            (uint8_t i2cAddress,
             bus_handlers::data::I2CBusID i2cBusID,
             OperatingModes operatingMode = NDOF,
             PowerModes powerMode = NORMAL,
             bool useExternalCrystal = true);

        /**
         * @brief Executes a software reset,
         * instead of a hardware-based reset that requires a physical disconnection.
         */
        bool reset() const;

        bool testWhoAmI() const;

        // ----------------------------
        // -- Calibration Operations --
        // ----------------------------
        // @{

        enum CalibrationLevels : uint8_t {
            UNCALIBRATED,
            PARTIALLY_CALIBRATED,
            FULLY_CALIBRATED
        };

        struct CalibrationStates {
            CalibrationLevels Platform;
            CalibrationLevels Gyroscope;
            CalibrationLevels Accelerometer;
            CalibrationLevels Magnetometer;
        };

        CalibrationStates getSensorsCalibration() const;

        bool calibrationComplete() const;

        // Offsets:

        struct SensorOffsetValues {
            int16_t accel_x, accel_y, accel_z;
            int16_t gyro_x , gyro_y , gyro_z;
            int16_t mgm_x  , mgm_y  , mgm_z;

            int16_t accel_radius;
            int16_t mgm_radius;
        };

        SensorOffsetValues getSensorOffsets();

        bool SetSensorsOffsets(const SensorOffsetValues &offsets);

        // Remapping:

        enum AxisRemappingSelection : uint8_t {
            REMAP_TO_X = 0U,
            REMAP_TO_Y = 1U,
            REMAP_TO_Z = 2U
        };

        struct AxesRemapping { AxisRemappingSelection x, y, z; };

        bool RemapAxes(const AxesRemapping remapping);

        enum AxisSignSelection : uint8_t {
            REMAP_TO_POSITIVE = 0U,
            REMAP_TO_NEGATIVE = 1U
        };

        struct AxesSigns { AxisSignSelection x, y, z; };

        bool RemapSigns(const AxesSigns remapping);

        struct AxisSignAndRemappingSelection {
            AxisSignSelection      sign;
            AxisRemappingSelection remapping;
        };

        struct AxesConfiguration { AxisSignAndRemappingSelection x, y, z; };

        AxesConfiguration getAxesConfiguration();

        // @}

        // -----------------------------------
        // -- Operating Modes Configuration --
        // -----------------------------------

        bool setMode(PowerModes mode) const;

        OperatingModes getCurrentMode() const;
        bool setMode(OperatingModes operatingMode);

        // ----------------------------------
        // -- Measurement Units Operations --
        // ----------------------------------
        // @{

        enum AccelerationUnits : uint8_t {
            METERS_SECOND_SQUARED = 0x00U,
            MILLIGRAVITY = 0x01U
        };

        bool setAccelerationUnits(AccelerationUnits units);

        AccelerationUnits getAccelerationUnits() const;


        enum AngularRateUnits : uint8_t {
            DEGREES_SECOND = 0x00U,
            RADIANS_SECOND = 0x02U
        };

        bool setAngularRateUnits(AngularRateUnits units);

        AngularRateUnits getAngularRateUnits() const;


        enum EulerAngleUnits : uint8_t {
            DEGREES = 0b00000000U,
            RADIANS = 0b00000100U
        };

        bool setEulerAngleUnits(EulerAngleUnits units);

        EulerAngleUnits getEulerAngleUnits() const;


        enum TemperatureUnits : uint8_t {
            CELSIUS = 0b00000000U,
            FAHRENHEIT = 0b00010000U,
        };

        bool setTemperatureUnits(TemperatureUnits units);

        TemperatureUnits getTemperatureUnits() const;


        enum PitchRotationConventions : uint8_t {
            CLOCKWISE_INCREASING = 0b00000000U,
            CLOCKWISE_DECREASING = 0b10000000U
        };

        bool setPitchRotation(PitchRotationConventions convention);

        PitchRotationConventions getPitchRotationConvention() const;

        // @}

        // ------------------------------------
        // -- Measurement Reading Operations --
        // ------------------------------------
        // @{

        /// @brief Base data type for sensor data readings
        struct SensorData { float X, Y, Z; };

        struct MagField : SensorData {};
        void readSensorData(MagField & data);

        struct Acceleration : SensorData {};
        void readSensorData(Acceleration &data);

        struct AngularVelocity : SensorData {};
        void readSensorData(AngularVelocity &data);

        struct Gravity : SensorData {};
        void readFusedData(Gravity &data);

        struct EulerOrientation : SensorData {};
        void readFusedData(EulerOrientation &data);

        struct LinearAcceleration : SensorData {};
        void readFusedData(LinearAcceleration &data);

        struct Quaternion { float W, X, Y, Z; };
        void readFusedData(Quaternion & data) const;

        struct FusedData {
            Gravity            gravity;
            EulerOrientation   eulerOrientation;
            LinearAcceleration linearAcceleration;
            Quaternion         quat;
        };
        void readAllFusedData(FusedData &data);

        struct Temperatures {
            std::int8_t from_accelerometer;
            std::int8_t from_gyroscope;
        };
        void readTemperatures(Temperatures &data);

        // @}

    private:
        OperatingModes mode {OperatingModes::CONFIG};
        bus_handlers::data::I2CBusID i2cBusID {bus_handlers::data::I2CBusID::BUS0};
        uint8_t i2cAddress;

        bool setUnitsRegister(uint8_t newValue, uint8_t mask);
        uint8_t getUnitsRegister(uint8_t unitsMask) const;
        
        void read3AxisMeasurement(uint8_t source, float lsb, SensorData & data);

        int16_t value_at(uint8_t source);

        enum TemperatureSource : uint8_t {
            ACCELEROMETER = 0U,
            GYROSCOPE     = 1U
        };
        int8_t readTemperatureFrom(TemperatureSource source);
    };
}

#endif //HAL_BNO055_IMU_H