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
//  Light.cpp
//  Stonefish
//
//  Created by Patryk Cieslak on 4/7/17.
//  Copyright (c) 2017-2023 Patryk Cieslak. All rights reserved.
//

#include "comms/Luma.h"
#include "graphics/OpenGLLight.h"

#include "core/GraphicalSimulationApp.h"


namespace sf
{

Luma::Luma(std::string uniqueName, uint64_t deviceId, Scalar r, Scalar cs): Comm(uniqueName, deviceId) {
    if(!SimulationApp::getApp()->hasGraphics())
        cCritical("Not possible to use lights in console simulation! Use graphical simulation if possible.");
    
    R=0.0005;
    //Fi = lum < Scalar(0) ? Scalar(0) : lum;
    range=r;
    comm_speed=cs;
    active=false;
}

    
CommType Luma::getType() const
{
    return CommType::OPTIC;
}


void Luma::SwitchOff(){
     for(int i=0;i<15;i++){
         lights[i]->getGLLight()->SwitchOff();
     }
}

void Luma::SwitchOn(){
     for(int i=0;i<15;i++){
         lights[i]->getGLLight()->SwitchOn();
     }
}

bool Luma::isActive(){
     return active;
}
        

void Luma::enable(bool t){
     active=t;
}

/*bool Luma::mutualContact(uint64_t device1Id, uint64_t device2Id)
{
    Luma* node1 = getNode(device1Id);
    Luma* node2 = getNode(device2Id);
    
    if(node1 == nullptr || node2 == nullptr)
        return false;
        
    Vector3 pos1 = node1->getDeviceFrame().getOrigin();
    Vector3 pos2 = node2->getDeviceFrame().getOrigin();
    Vector3 dir = pos2-pos1;
    Scalar distance = dir.length();
    
    if(!node1->isReceptionPossible(dir, distance) || !node2->isReceptionPossible(-dir, distance))
        return false;
        
    if(node1->getOcclusionTest() || node2->getOcclusionTest())
    {
        btCollisionWorld::ClosestRayResultCallback closest(pos1, pos2);
        closest.m_collisionFilterGroup = MASK_DYNAMIC;
        closest.m_collisionFilterMask = MASK_STATIC | MASK_DYNAMIC | MASK_ANIMATED_COLLIDING;
        SimulationApp::getApp()->getSimulationManager()->getDynamicsWorld()->rayTest(pos1, pos2, closest);
        return !closest.hasHit();
    }
    else
        return true;
}*/

void Luma::InternalUpdate(Scalar dt)
{
    //Propagate messages already sent
    /*std::map<AcousticDataFrame*, Vector3>::iterator mIt;
    for(mIt = propagating.begin(); mIt != propagating.end(); )
    {
        AcousticModem* dest = getNode(mIt->first->destination);
        Vector3 dO = dest->getDeviceFrame().getOrigin();
        Vector3 sO = mIt->second;
        Vector3 dir = dO - sO;
        Scalar d = dir.length();
        
        if(d <= SOUND_VELOCITY_WATER*dt) //Message reached?
        {
            mIt->first->travelled += d;
            dest->MessageReceived(mIt->first);
            mIt = propagating.erase(mIt);
        }
        else //Advance pulse
        {
            dir /= d; //Normalize direction
            d = SOUND_VELOCITY_WATER * dt;
            mIt->second += dir * d;
            mIt->first->travelled += d;
            ++mIt;
        }
    }
    
    //Send first message from the tx buffer
    if(txBuffer.size() > 0)
    {
        AcousticDataFrame* msg = (AcousticDataFrame*)txBuffer[0];
        if(mutualContact(msg->source, msg->destination))
            propagating[msg] = msg->txPosition;
        else
            delete msg;
            
        txBuffer.pop_front();
    }*/
}

void Luma::ProcessMessages()
{
    /*AcousticDataFrame* msg;
    while((msg = (AcousticDataFrame*)ReadMessage()) != nullptr)
    {
        //Different responses to messages should be implemented here
        if(msg->data != "ACK")
        {
            //timestamp and sequence don't change
            msg->destination = msg->source;
            msg->source = getDeviceId();
            msg->data = "ACK";
            msg->txPosition = getDeviceFrame().getOrigin();
            txBuffer.push_back(msg);
        }
        else
        {
            delete msg;
        }
    }*/
}
    
std::vector<Renderable> Luma::Render()
{
    std::vector<Renderable> items(0);
    /*
    //glLight->SwitchOff();
    Renderable item;
    item.model = glMatrixFromTransform(getActuatorFrame());
    item.type = RenderableType::ACTUATOR_LINES;
    
    GLfloat iconSize = 1.f;
    unsigned int div = 24;
    
    if(coneAngle > Scalar(0))
    {
        GLfloat r = iconSize * tanf((GLfloat)coneAngle/360.f*M_PI);
        
        for(unsigned int i=0; i<div; ++i)
        {
            GLfloat angle1 = (GLfloat)i/(GLfloat)div * 2.f * M_PI;
            GLfloat angle2 = (GLfloat)(i+1)/(GLfloat)div * 2.f * M_PI;
            item.points.push_back(glm::vec3(r * cosf(angle1), r * sinf(angle1), iconSize));
            item.points.push_back(glm::vec3(r * cosf(angle2), r * sinf(angle2), iconSize));
        }
        
        item.points.push_back(glm::vec3(0,0,0));
        item.points.push_back(glm::vec3(r, 0, iconSize));
        item.points.push_back(glm::vec3(0,0,0));
        item.points.push_back(glm::vec3(-r, 0, iconSize));
        item.points.push_back(glm::vec3(0,0,0));
        item.points.push_back(glm::vec3(0, r, iconSize));
        item.points.push_back(glm::vec3(0,0,0));
        item.points.push_back(glm::vec3(0, -r, iconSize));
    }
    else
    {
        for(unsigned int i=0; i<div; ++i)
        {
            GLfloat angle1 = (GLfloat)i/(GLfloat)div * 2.f * M_PI;
            GLfloat angle2 = (GLfloat)(i+1)/(GLfloat)div * 2.f * M_PI;
            item.points.push_back(glm::vec3(0.5f * iconSize * cosf(angle1), 0.5f * iconSize * sinf(angle1), 0));
            item.points.push_back(glm::vec3(0.5f * iconSize * cosf(angle2), 0.5f * iconSize * sinf(angle2), 0));
            item.points.push_back(glm::vec3(0.5f * iconSize * cosf(angle1), 0, 0.5f * iconSize * sinf(angle1)));
            item.points.push_back(glm::vec3(0.5f * iconSize * cosf(angle2), 0, 0.5f * iconSize * sinf(angle2)));
            item.points.push_back(glm::vec3(0, 0.5f * iconSize * cosf(angle1), 0.5f * iconSize * sinf(angle1)));
            item.points.push_back(glm::vec3(0, 0.5f * iconSize * cosf(angle2), 0.5f * iconSize * sinf(angle2)));
        }
    }
    
    items.push_back(item);
    */
    return items;
}

std::vector<Light*> Luma::getLights(){
    return lights;
}

void Luma::addLight(Light* l){
    return lights.push_back(l);
}


Scalar Luma::getRange(){
       return range;
}
        
void Luma::setRange(Scalar r){
     range=r;
}
        
Scalar Luma::getCommSpeed(){
       return comm_speed;
}
        
void Luma::setCommSpeed(Scalar cs){
     comm_speed=cs;
}

}
