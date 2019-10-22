#ifndef SISO_CONTROLLERS_H
#define SISO_CONTROLLERS_H

#include <cstdio>
#include <utils/linalg/linalg.h>
#include <signal/systemmodels/systemmodels.hpp>
#include <mbed.h>

namespace signal{
namespace controllers
{
    namespace siso
    {
        
        /**
         * @brief General interface class for the controller with single input and single output
         * 
         * @tparam T type of the variables (float, double)
         */
        template<class T>
        class IController{
            public:
                virtual T calculateControl(const T&)=0;
                virtual void clear()=0;
        };

        /**
         * @brief It generates a discrete transferfunction for realizing a proportional–integral–derivative controller, which is discretized by the Euler’s method.
         * 
         * @tparam T type of the variables (float, double) 
         */
        template<class T>
        class CPidController:public IController<T>
        {
            public:
                /* Discrete transferfunction type */

                /** @brief Type of the system model for a pid controller */
                using CPidSystemmodelType = signal::systemmodels::lti::siso::CDiscreteTransferFunction<T,3,3>;
                
                /* Constructor */
                CPidController(T              f_kp
                              ,T              f_ki
                              ,T              f_kd
                              ,T              f_tf
                              ,T              f_dt);
                CPidController(CPidSystemmodelType f_pid,T f_dt);

                /* Calculate the control signal based the input error. */
                T calculateControl(const T& f_input);
                /* Serial callback implementation */
                void serialCallback(char const * a, char * b);

                /* Set to zero the previous values in the transferfunction  */
                void clear();
            private:
                /* Set the controller's parameters. */
                void setController(T              f_kp
                                  ,T              f_ki
                                  ,T              f_kd
                                  ,T              f_tf);

                /* Discrete transferfunction */
                CPidSystemmodelType     m_pidTf;


                /* Sampling time */
                T                 m_dt;

        };
        /* Include function definitions */
        #include "sisocontrollers.tpp"
    }; // namespace siso
}; // namespace controllers   

}; //namespace signal


#endif