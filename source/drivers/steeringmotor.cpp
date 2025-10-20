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

#include <drivers/steeringmotor.hpp>

namespace drivers{
    /**
     * @brief It initializes the pwm parameters and it sets the steering in zero position, the limits of the input degree value.
     * 
     * @param f_pwm               pin connected to servo motor
     * @param f_inf_limit         inferior limit 
     * @param f_sup_limit         superior limit
     * 
     */
    CSteeringMotor::CSteeringMotor(
            PinName f_pwm_pin, 
            int f_inf_limit, 
            int f_sup_limit
        )
        :m_pwm_pin(f_pwm_pin)
        ,m_inf_limit(f_inf_limit)
        ,m_sup_limit(f_sup_limit)
    {
        // Set the ms_period on the pwm_pin
        m_pwm_pin.period_ms(ms_period);
        // Set position to zero (center position at 1.5ms)
        m_pwm_pin.pulsewidth_us(center_pulsewidth_us);
        ThisThread::sleep_for(chrono::milliseconds(10)); 
    };


    /** @brief  CSteeringMotor class destructor
     */
    CSteeringMotor::~CSteeringMotor()
    {
    };
    
    /** @brief  It modifies the angle of the servo motor, which controls the steering wheels. 
     *
     *  @param f_angle      angle degree, where the positive value means right direction and negative value the left direction. 
     */
    void CSteeringMotor::setAngle(int f_angle)
    {
        m_pwm_pin.pulsewidth_us(conversion(f_angle));
    };

    /** @brief  It converts angle degree to pulse width in microseconds for pwm signal. 
     * 
     *  @param f_angle    angle degree
     *  \return         pulse width in microseconds
     */
    uint16_t CSteeringMotor::conversion(int f_angle)
    {
        // Clamp angle to range
        if (f_angle > m_sup_limit) f_angle = m_sup_limit;
        if (f_angle < m_inf_limit) f_angle = m_inf_limit;

        // Linear mapping: -MAXdeg -> 1000us, 0deg -> 1500us, +MAXdeg -> 2000us
        // pulsewidth = center + (angle / max_angle) * (max_pulse - center)
        return center_pulsewidth_us + (f_angle * (max_pulsewidth_us - center_pulsewidth_us)) / m_sup_limit;
    };
//
    /**
     * @brief It verifies whether a number is in a given range
     * 
     * @param f_angle value 
     * @return true means, that the value is in the range
     * @return false means, that the value isn't in the range
     */
    bool CSteeringMotor::inRange(int f_angle){
        return m_inf_limit<=f_angle && f_angle<=m_sup_limit;
    };
}; // namespace hardware::drivers