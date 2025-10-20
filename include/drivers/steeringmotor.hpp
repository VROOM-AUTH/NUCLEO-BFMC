/**
 * Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC organizers
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:

 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.

 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.

 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.

 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
*/

/* Include guard */
#ifndef STEERINGMOTOR_HPP
#define STEERINGMOTOR_HPP

#include <mbed.h>
#include <utility>
#include <cmath>
#include <climits>
#include <chrono>

namespace drivers
{
    /**
     * @brief Interface to control the steering angle
     * 
     */
    class ISteeringCommand
    {
        public:
            virtual void setAngle(int f_angle) = 0 ;
            virtual bool inRange(int f_angle) = 0 ;
    };


    /**  
     * @brief Steering servo motor driver
     * 
     * It is used to control the servo motor, which is connected to the steering wheels. The steering angle can be accessed through 'setAngle' method. 
     * 
     */
    class CSteeringMotor: public ISteeringCommand
    {
        public:
            /* Constructor */
            CSteeringMotor(
                PinName f_pwm_pin,
                int f_inf_limit,
                int f_sup_limit
            );
            /* Destructor */
            ~CSteeringMotor();
            /* Set angle */
            void setAngle(int f_angle); 
            /* Check if angle in range */
            bool inRange(int f_angle);
        private:
            /** @brief PWM output pin */
            PwmOut m_pwm_pin;
            /** @brief PWM period in milliseconds */
            const int8_t ms_period = 20; // 20ms period
            /** @brief Minimum pulse width in microseconds (for -30 deg) */
            const uint16_t min_pulsewidth_us = 1000; // 1ms
            /** @brief Maximum pulse width in microseconds (for +30 deg) */
            const uint16_t max_pulsewidth_us = 2000; // 2ms
            /** @brief Center pulse width in microseconds (for 0 deg) */
            const uint16_t center_pulsewidth_us = 1500; // 1.8ms
            /** @brief Inferior limit */
            const int m_inf_limit;
            /** @brief Superior limit */
            const int m_sup_limit;

            /* convert angle degree to pulse width in microseconds for pwm signal */
            uint16_t conversion(int f_angle);
    }; // class ISteeringCommand
}; // namespace drivers


#endif //STEERINGMOTOR_HPP