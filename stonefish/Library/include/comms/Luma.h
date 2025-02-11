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
//  Luma.h
//  Stonefish

#ifndef __Stonefish_Luma__
#define __Stonefish_Luma__

#include "actuators/Light.h"
#include "comms/Comm.h"
#include "graphics/OpenGLDataStructs.h"

namespace sf
{
    class Light;
	class StaticEntity;
    class AnimatedEntity;
    
    //! A class representing a light of two common types: omni and spot.
    class Luma : public Comm
    {
    public:
        //! A constructor of an omni light.
        /*!
         \param uniqueName a name of the light
		 \param radius a radius of the light source [m]
         \param color a color of the light
         \param lum the luminous power of the light [lm]
         */
        Luma(std::string uniqueName, uint64_t deviceId, Scalar range, Scalar comm_speed);
        
        //! A constructor of a spot light.
        /*!
         \param uniqueName a name of the light
		 \param radius a radius of the light source [m]
         \param coneAngleDeg a cone angle of the spot light in degrees [deg]
         \param color a color of the light
         \param lum the luminous power of the light [lm]
         */
        
		//! A method used to attach the comm device to the world origin.
        /*!
         \param origin the place where the comm should be attached in the world frame
         */
        /*!
         \param dt a time step of the simulation
         */
        virtual void InternalUpdate(Scalar dt);
        
        //! A method implementing the rendering of the light dummy.
        std::vector<Renderable> Render();
        
        //! A method returning the type of the actuator.
        CommType getType() const;
        
        std::vector<Light*> getLights();
        
        void SwitchOff();
        
        void SwitchOn();
        
        bool isActive();
        
        void enable(bool t);
        
        Scalar getRange();
        
        void setRange(Scalar r);
        
        Scalar getCommSpeed();
        
        void setCommSpeed(Scalar r);
        
        void addLight(Light* l);
        
        
    protected:
    
        virtual void ProcessMessages();
        
    private:
        
	Scalar R;
        Scalar Fi;
        Scalar coneAngle;
        std::vector<Light*> lights;
        bool active;
        Scalar range; 
        Scalar comm_speed;
    };
}

#endif
