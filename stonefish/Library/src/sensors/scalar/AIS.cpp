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
//  GPS.cpp
//  Stonefish
//
//  Created by Patryk Cieslak on 02/11/2017.
//  Copyright (c) 2017-2021 Patryk Cieslak. All rights reserved.
//

#include "sensors/scalar/AIS.h"

#include "core/SimulationApp.h"
#include "core/SimulationManager.h"
#include "core/NED.h"
#include "entities/forcefields/Ocean.h"
#include "sensors/Sample.h"

namespace sf
{

AIS::AIS(std::string uniqueName, Scalar frequency, int historyLength) : LinkSensor(uniqueName, frequency, historyLength)
{
    ID="";
    channels.push_back(SensorChannel("Latitude", QuantityType::ANGLE));
    channels.push_back(SensorChannel("Longitude", QuantityType::ANGLE));
    channels.push_back(SensorChannel("North", QuantityType::LENGTH));
    channels.push_back(SensorChannel("East", QuantityType::LENGTH));
    setGPSNoise(Scalar(0));
    channels.push_back(SensorChannel("Position X", QuantityType::LENGTH));
    channels.push_back(SensorChannel("Position Y", QuantityType::LENGTH));
    channels.push_back(SensorChannel("Position Z", QuantityType::LENGTH));
    channels.push_back(SensorChannel("Velocity X", QuantityType::VELOCITY));
    channels.push_back(SensorChannel("Velocity Y", QuantityType::VELOCITY));
    channels.push_back(SensorChannel("Velocity Z", QuantityType::VELOCITY));
    channels.push_back(SensorChannel("Orientation X", QuantityType::UNITLESS));
    channels.push_back(SensorChannel("Orientation Y", QuantityType::UNITLESS));
    channels.push_back(SensorChannel("Orientation Z", QuantityType::UNITLESS));
    channels.push_back(SensorChannel("Orientation W", QuantityType::UNITLESS));
    channels.push_back(SensorChannel("Angular velocity X", QuantityType::ANGULAR_VELOCITY));
    channels.push_back(SensorChannel("Angular velocity Y", QuantityType::ANGULAR_VELOCITY));
    channels.push_back(SensorChannel("Angular velocity Z", QuantityType::ANGULAR_VELOCITY));
    channels.push_back(SensorChannel("Heading", QuantityType::ANGLE));
    ornStdDev = Scalar(0);
    ornNoise = std::normal_distribution<Scalar>(Scalar(0), ornStdDev);
    
}

void AIS::InternalUpdate(Scalar dt)
{
    Scalar yaw, pitch, roll;
    getSensorFrame().getBasis().getEulerYPR(yaw, pitch, roll);
    //get sensor frame in world
    Transform aisTrans = getSensorFrame();
    //Calculate transformation from global to imu frame
   
    Vector3 pos = aisTrans.getOrigin();
    Vector3 v = aisTrans.getBasis().inverse() * attach->getLinearVelocityInLocalPoint(aisTrans.getOrigin() - attach->getCGTransform().getOrigin());
    
    Quaternion orn = aisTrans.getRotation();
    Scalar angle = orn.getAngle() + ornNoise(randomGenerator);
    orn = Quaternion(orn.getAxis(), angle);

    Vector3 av = aisTrans.getBasis().inverse() * attach->getAngularVelocity();
    
    //GPS not updating underwater
    Ocean* liq = SimulationApp::getApp()->getSimulationManager()->getOcean();
    if(liq != nullptr && liq->IsInsideFluid(aisTrans.getOrigin()))
    {
        Scalar data[18] = {BT_LARGE_FLOAT, 
                          BT_LARGE_FLOAT, 
                          Scalar(0), Scalar(0),pos.x(), pos.y(), pos.z(), v.x(), v.y(), v.z(), orn.x(), orn.y(), orn.z(), orn.w(), av.x(), av.y(), av.z()};
        Sample s(18, data);
        AddSampleToHistory(s);
    }
    else
    {
        Vector3 gpsPos = aisTrans.getOrigin();
		
        //add noise
        if(!btFuzzyZero(nedStdDev))
        {
            gpsPos.setX(gpsPos.x() + noise(randomGenerator));
            gpsPos.setY(gpsPos.y() + noise(randomGenerator));
        }
        
        //convert NED to geodetic coordinates
        double latitude;
        double longitude;
        double height;
        SimulationApp::getApp()->getSimulationManager()->getNED()->Ned2Geodetic(gpsPos.x(), gpsPos.y(), 0.0, latitude, longitude, height);
        
        //record sample
        Scalar data[18] = {latitude, longitude, gpsPos.x(), gpsPos.y(), pos.x(), pos.y(), pos.z(), v.x(), v.y(), v.z(), orn.x(), orn.y(), orn.z(), orn.w(), av.x(), av.y(), av.z()};
        Sample s(18, data);
        AddSampleToHistory(s);
    }
    
    
    Scalar heading = yaw * (180.0 / M_PI); // Convert yaw from radians to degrees

    // Compute Speed Through Water (STW)
    stw = std::sqrt(v.x() * v.x() + v.y() * v.y() + v.z() * v.z()); // Speed through Water in knots, calculated from velocity vector

    // Assuming the ocean current can be retrieved like this:
    Vector3 currentVelocity = liq->GetFluidVelocity(aisTrans.getOrigin());
    Scalar cs = std::sqrt(currentVelocity.x() * currentVelocity.x() + currentVelocity.y() * currentVelocity.y() + currentVelocity.z() * currentVelocity.z()); // Current Speed in knots
    
    Scalar csKnots = cs * 1.94384; // Conversion factor from m/s to knots
    
    Scalar cd = atan2(currentVelocity.y(), currentVelocity.x()) * (180.0 / M_PI); // Current Direction in degrees

    Scalar headingRad = heading * (M_PI / 180.0);
    Scalar currentDirectionRad = cd * (M_PI / 180.0);

    Scalar vx = stw * cos(headingRad);
    Scalar vy = stw * sin(headingRad);

    Scalar cx = csKnots * cos(currentDirectionRad);
    Scalar cy = csKnots * sin(currentDirectionRad);

    Scalar tx = vx + cx;
    Scalar ty = vy + cy;

    Scalar cogRad = atan2(ty, tx);
    cog = fmod(cogRad * (180.0 / M_PI) + 360.0, 360.0);

    sog = std::sqrt(tx * tx + ty * ty);
    
    rot=av.z();
    
    
    
}

void AIS::setGPSNoise(Scalar nedDev)
{
    nedStdDev = btClamped(nedDev, Scalar(0), Scalar(BT_LARGE_FLOAT));
    noise = std::normal_distribution<Scalar>(Scalar(0), nedStdDev);
}

void AIS::setCompassNoise(Scalar headingStdDev)
{
    channels[18].setStdDev(btClamped(headingStdDev, Scalar(0), Scalar(BT_LARGE_FLOAT)));
}

void AIS::setOrnNoise(Scalar positionStdDev, Scalar velocityStdDev, Scalar angleStdDev, Scalar angularVelocityStdDev)
{
    channels[4].setStdDev(btClamped(positionStdDev, Scalar(0), Scalar(BT_LARGE_FLOAT)));
    channels[5].setStdDev(btClamped(positionStdDev, Scalar(0), Scalar(BT_LARGE_FLOAT)));
    channels[6].setStdDev(btClamped(positionStdDev, Scalar(0), Scalar(BT_LARGE_FLOAT)));
    channels[7].setStdDev(btClamped(velocityStdDev, Scalar(0), Scalar(BT_LARGE_FLOAT)));
    channels[8].setStdDev(btClamped(velocityStdDev, Scalar(0), Scalar(BT_LARGE_FLOAT)));
    channels[9].setStdDev(btClamped(velocityStdDev, Scalar(0), Scalar(BT_LARGE_FLOAT)));
    channels[15].setStdDev(btClamped(angularVelocityStdDev, Scalar(0), Scalar(BT_LARGE_FLOAT)));
    channels[16].setStdDev(btClamped(angularVelocityStdDev, Scalar(0), Scalar(BT_LARGE_FLOAT)));
    channels[17].setStdDev(btClamped(angularVelocityStdDev, Scalar(0), Scalar(BT_LARGE_FLOAT)));
    ornStdDev = btClamped(angleStdDev, Scalar(0), Scalar(BT_LARGE_FLOAT));
    ornNoise = std::normal_distribution<Scalar>(Scalar(0), ornStdDev);
}

Scalar AIS::getGPSNoise()
{
    return nedStdDev;
}

ScalarSensorType AIS::getScalarSensorType()
{
    return ScalarSensorType::AIS;
}

std::string AIS::getID(){
     return ID;
}
void AIS::setID(std::string s){ 
     ID=s;
}

std::string AIS::getVesselType(){
     return vessel_type;
}
void AIS::setVesselType(std::string s){ 
     vessel_type=s;
}
}
