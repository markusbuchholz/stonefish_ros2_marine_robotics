/*    
    This file is a part of Stonefish.

    Stonefish is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Stonefish is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

//
//  DCMotor.h
//  Stonefish
//
//  Created by Patryk Cieslak on 1/11/13.
//  Copyright (c) 2013-2018 Patryk Cieslak. All rights reserved.
//

#ifndef __Stonefish_DCMotor__
#define __Stonefish_DCMotor__

#include "actuators/Motor.h"

namespace sf
{
    //! A class representing a model of a DC motor.
    class DCMotor : public Motor
    {
    public:
        //! A constructor.
        /*!
         \param uniqueName a name for the motor
         \param motorR the resistance of the motor [Ohm]
         \param motorL the inductance of the motor [H]
         \param motorKe the speed constant of the motor [V/rpm]
         \param motorKt the torque constant of the motor [Nm/A]
         \param friction the friction coefficient
         */
        DCMotor(std::string uniqueName, Scalar motorR, Scalar motorL, Scalar motorKe, Scalar motorKt, Scalar friction);
        
        //! A method used to update the internal state of the motor.
        /*!
         \param dt a time step of the simulation [s]
         */
        void Update(Scalar dt);
        
        //! A method to setup a simulated gearbox connected to the motor.
        /*!
         \param enable a flag to indicate if the gearbox should be enabled
         \param ratio a gear ratio
         \param efficiency an efficiency factor
         */
        void SetupGearbox(bool enable, Scalar ratio, Scalar efficiency);
        
        //! A method to set the voltage driving the motor.
        /*!
         \param volt the voltage at the motor terminals [V]
         */
        void setIntensity(Scalar volt);
        
        //! A method returning the torque generated by the motor.
        Scalar getTorque() const;
        
        //! A method returning the angular position of the motor.
        Scalar getAngle() const;
        
        //! A method returning the angular velocity of the motor.
        Scalar getAngularVelocity() const;
        
        //! A method returning the current in the motor windings.
        Scalar getCurrent() const;
        
        //! A method returning the voltage at the motor terminals.
        Scalar getVoltage() const;
        
        //! A method returning the resistance of the motor windings.
        Scalar getR() const;
        
        //! A method returning the inductance of the motor windings.
        Scalar getL() const;
        
        //! A method returning the velocity constant of the motor.
        Scalar getKe() const;
        
        //! A method returning the torque constant of the motor.
        Scalar getKt() const;
        
        //! A method returning the ratio of the motor gearbox.
        Scalar getGearRatio() const;
        
    private:
        Scalar V;
        Scalar I;
        Scalar R;
        Scalar L;
        Scalar Ke;
        Scalar Kt;
        Scalar B;
        bool gearEnabled;
        Scalar gearRatio;
        Scalar gearEff;
        Scalar lastVoverL;
    };
}

#endif