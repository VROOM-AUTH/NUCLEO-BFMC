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

#include "brain/klmanager.hpp"

#define _25_chars 25

// TODO: Add your code here
namespace brain
{
   /**
    * @brief Class constructorklmanager
    *
    */
    CKlmanager::CKlmanager(
        brain::CRobotStateMachine& f_robotStateMachine,
        periodics::CResourcemonitor& f_resourceM
    )
    : m_klvalue(0)
    , m_robotStateMachine(f_robotStateMachine)
    , m_resourceM(f_resourceM)
    {
        /* constructor behaviour */
    }

    /** @brief  CKlmanager class destructor
     */
    CKlmanager::~CKlmanager()
    {
    };

    void CKlmanager::serialCallbackKLCommand(const char* a, char* b)
    {
        uint8_t l_keyValue = 0;

        (void)sscanf(a,"%hhu",&l_keyValue);

        if(!bool_globalsV_ShuttedDown)
        {
            if(l_keyValue == 15 || l_keyValue == 30 || l_keyValue == 0)
            {
                CKlmanager::m_klvalue = l_keyValue;

                char response[_25_chars];

                if(l_keyValue == 30 && (uint8_globalsV_value_of_kl != 30)){
                    sprintf(b,"%d",l_keyValue);
                    uint8_globalsV_value_of_kl = 30;
                    m_robotStateMachine.serialCallbackVCDcommand("0;0;2", response);
                }
            }
            else{
                sprintf(b,"syntax error");
            }
        }
    }

}; // namespace brain