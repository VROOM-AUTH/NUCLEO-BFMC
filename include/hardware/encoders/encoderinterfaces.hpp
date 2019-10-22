/**
 * @file encoderinterface.hpp
 * @author RBRO/PJ-IU
 * @brief 
 * @version 0.1
 * @date 2018-10-24
 * 
 * @copyright Copyright (c) 2018
 * 
 */
#ifndef ENDOCER_INTERFACE_HPP
#define ENDOCER_INTERFACE_HPP

#include <stdint.h>
namespace hardware::encoders{
    /**
     * @brief Rotary speed encoder interface class.
     *
     */
    class IEncoderGetter{
        public:
        /**
         * @brief Get the counted impulse in the last period. It can be positive or negative number, when it uses by quadrature encoder.
         * 
         * @return Rotation speed in a period
         */
        virtual int16_t  getCount() = 0;
        /**
         * @brief Get the rotation speed of the encoder in the last period based the predefined period and encoder resolution. The result is returned in rotation per second (rps).
         * 
         * @return Counted value in a period
         */
        virtual float getSpeedRps() = 0;
        /**
         * @brief Get the encoder capacity. If it's false, than the encoder can give the orientation like a positive or a negative rotation speed, else it returns only the absolute value of the rotation speed. 
         * 
         * @return Capacity of measuring the rotation direction
         */
        virtual bool isAbs() = 0;
    };


    
    /**
     * @brief Rotary speed encoder interface class for accessing non-filtered values
     * 
     * This interface is used when a filter is attached to the encoder and the initial interface 'IEncoderGetter' returns the filtered values. In this case, this interface ensure the access to the non-filtered values. 
     */
    class IEncoderNonFilteredGetter{
        public:
        /**
         * @brief Get the non-filtered counted impulse in the last period. 
         * 
         * @return Non-filtered counted value.
         */
        virtual int16_t  getNonFilteredCount()=0;
        /**
         * @brief Get the non-filtered rotation speed [rotation per second]
         * 
         * @return Non-filtered rotation speed
         */
        virtual float getNonFilteredSpeedRps() = 0;
    };
}; // namepsace hardware::encoders;

#endif