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
//  GraphicalSimulationApp.cpp
//  Stonefish
//
//  Created by Patryk Cieslak on 11/28/12.
//  Copyright (c) 2012-2023 Patryk Cieslak. All rights reserved.
//

#include "core/GraphicalSimulationApp.h"
#include "stb_image_write.h"
#include <chrono>
#include <thread>
#include <omp.h>
#include "core/SimulationManager.h"
#include "core/Robot.h"
#include "sensors/Sample.h"
#include "graphics/OpenGLState.h"
#include "graphics/GLSLShader.h"
#include "graphics/OpenGLPipeline.h"
#include "graphics/OpenGLConsole.h"
#include "graphics/IMGUI.h"
#include "graphics/OpenGLTrackball.h"
#include "utils/SystemUtil.hpp"
#include "entities/Entity.h"
#include "entities/StaticEntity.h"
#include "entities/SolidEntity.h"
#include "entities/MovingEntity.h"
#include "entities/solids/Compound.h"
#include "utils/icon.h"
#include <iostream>
#include "sensors/Sensor.h"
#include "sensors/scalar/GPS.h"
#include <string>
#include <fstream>
#include <sys/types.h>
#include <sys/stat.h>
#include <sstream>
#include <filesystem>
#include <fstream>
#include <ctime>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include <cstdlib>    // For _mkdir in Windows


namespace sf
{

std::vector<unsigned char> pixels(400 * 400 * 3);

cv::Vec3b generateColor(int entityId) {
    // Use bitwise operations to introduce randomness
    int r = (entityId * 131 % 256); // You can choose any multiplier
    int g = (entityId * 173 % 256); // You can choose any multiplier
    int b = (entityId * 193 % 256); // You can choose any multiplier

    // Return the generated color
    return cv::Vec3b(static_cast<uchar>(b), static_cast<uchar>(g), static_cast<uchar>(r)); // Return BGR color
}

cv::Vec3b getRandomColor() {
    return cv::Vec3b(rand() % 256, rand() % 256, rand() % 256);
}

GraphicalSimulationApp::GraphicalSimulationApp(std::string name, std::string dataDirPath, RenderSettings r, HelperSettings h,  std::string turbidity_lvl, SimulationManager* sim)
: SimulationApp(name, dataDirPath, sim)
{
#ifdef SHADER_DIR_PATH
    shaderPath = SHADER_DIR_PATH;
#else
    shaderPath = "/usr/local/share/Stonefish/shaders/";
#endif
    glLoadingContext = NULL;
    glMainContext = NULL;
    trackballCenter = NULL;
    selectedEntity = std::make_pair(nullptr, -1);
    displayHUD = true;
    displayKeymap = false;
    displayConsole = false;
    displayPerformance = false;
    joystick = NULL;
    joystickAxes = NULL;
    joystickButtons = NULL;
    joystickHats = NULL;
    mouseWasDown.type = SDL_LASTEVENT;
    limitFramerate = true;
    simulationThread = NULL;
    loadingThread = NULL;
    glPipeline = NULL;
    gui = NULL;
    timeQuery[0] = 0;
    timeQuery[1] = 0;
    timeQueryPingpong = 0;
    drawingTime = 0.0;
    maxDrawingTime = 0.0;
    maxCounter = 0;
    rSettings = r;
    hSettings = h;
    //windowW2 = 0;
    rSettings.windowW += rSettings.windowW % 2;
    rSettings.windowH += rSettings.windowH % 2;
    windowW = rSettings.windowW;
    windowH = rSettings.windowH;
    turbidity_level=std::stoi(turbidity_lvl);
    if(turbidity_level!=-1){
	    std::ostringstream text;
	    std::string oceanOpsShader=shaderPath+"oceanOptics.frag";
	    const char * ocean_optics_sh = oceanOpsShader.c_str();
	    chmod(ocean_optics_sh, S_IRWXU);
	    std::ifstream in_file(shaderPath+"oceanOptics.frag");
	    text << in_file.rdbuf();
	    std::string str = text.str();
	    std::string str_search = "Sfactor=10;";
	    std::string str_replace = "Sfactor="+turbidity_lvl+";";
	    size_t pos = str.find(str_search);
	    if(pos != std::string::npos){
	       str.replace(pos, std::string(str_search).length(), str_replace);
	    }
	    in_file.close();
	    std::ofstream out_file(shaderPath+"oceanOptics.frag");
	    out_file << str; 
    }
    
}

GraphicalSimulationApp::~GraphicalSimulationApp()
{
    if(console != NULL) delete console;
    if(glPipeline != NULL) delete glPipeline;
    if(gui != NULL) delete gui;
    
    if(joystick != NULL)
    {
        delete [] joystickButtons;
        delete [] joystickAxes;
        delete [] joystickHats;
    }
}

void GraphicalSimulationApp::ShowHUD()
{
    displayHUD = true;
}

void GraphicalSimulationApp::HideHUD()
{
    displayHUD = false;
}

void GraphicalSimulationApp::ShowConsole()
{
    displayConsole = true;
}

void GraphicalSimulationApp::HideConsole()
{
    displayConsole = false;
}

void GraphicalSimulationApp::setLimitFramerate(bool enabled)
{
    limitFramerate = enabled;
}

OpenGLPipeline* GraphicalSimulationApp::getGLPipeline()
{
    return glPipeline;
}

IMGUI* GraphicalSimulationApp::getGUI()
{
    return gui;
}

std::pair<Entity*, int> GraphicalSimulationApp::getSelectedEntity()
{
    return selectedEntity;
}

bool GraphicalSimulationApp::hasGraphics()
{
    return true;
}

SDL_Joystick* GraphicalSimulationApp::getJoystick()
{
    return joystick;
}

double GraphicalSimulationApp::getDrawingTime(bool max)
{
    if(max)
        return maxDrawingTime;
    else
        return drawingTime;
}

int GraphicalSimulationApp::getWindowWidth()
{
    return windowW;
}

int GraphicalSimulationApp::getWindowHeight()
{
    return windowH;
}

std::string GraphicalSimulationApp::getShaderPath()
{
    return shaderPath;
}

RenderSettings GraphicalSimulationApp::getRenderSettings() const
{
    return glPipeline->getRenderSettings();
}

HelperSettings& GraphicalSimulationApp::getHelperSettings()
{
    return glPipeline->getHelperSettings();
}

void GraphicalSimulationApp::Init()
{
    //General initialization
    cInfo("Initializing rendering pipeline:");
    SimulationApp::Init();
    //Window initialization + loading thread
    loading = true;
    cInfo("Initializing rendering pipeline:");
    InitializeSDL();

    //Continue initialization with console visible
    cInfo("Initializing rendering pipeline:");
    cInfo("Loading GUI...");
    gui = new IMGUI(windowW, windowH);
    InitializeGUI(); //Initialize non-standard graphical elements
    glPipeline = new OpenGLPipeline(rSettings, hSettings);
    ShowHUD();
    
    cInfo("Initializing simulation:");
    InitializeSimulation();
    
    cInfo("Ready for running...");
    SDL_Delay(1000);
    
    //Close loading console - exit loading thread
    loading = false;
    int status = 0;
    SDL_WaitThread(loadingThread, &status);
    SDL_GL_MakeCurrent(window, glMainContext);
    getSimulationManager()->setAnnotation(false);
    //Create performance counters
    glGenQueries(2, timeQuery);
}

void GraphicalSimulationApp::InitializeSDL()
{
    SDL_Init(SDL_INIT_VIDEO | SDL_INIT_EVENTS | SDL_INIT_JOYSTICK);
    
    //Create OpenGL contexts
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 4);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
    SDL_GL_SetAttribute(SDL_GL_STENCIL_SIZE, 8);
    SDL_GL_SetAttribute(SDL_GL_RED_SIZE, 8);
    SDL_GL_SetAttribute(SDL_GL_GREEN_SIZE, 8);
    SDL_GL_SetAttribute(SDL_GL_BLUE_SIZE, 8);
    SDL_GL_SetAttribute(SDL_GL_SHARE_WITH_CURRENT_CONTEXT, 1);
    
    //Create window
    window = SDL_CreateWindow(getName().c_str(),
                              SDL_WINDOWPOS_CENTERED,
                              SDL_WINDOWPOS_CENTERED,
                              windowW,
                              windowH,
                              SDL_WINDOW_OPENGL | SDL_WINDOW_SHOWN// | SDL_WINDOW_ALLOW_HIGHDPI
                              );
                              
    //Set window icon
    uint32_t rmask, gmask, bmask, amask;
#if SDL_BYTEORDER == SDL_BIG_ENDIAN
    int shift = (icon_image.bytes_per_pixel == 3) ? 8 : 0;
    rmask = 0xff000000 >> shift;
    gmask = 0x00ff0000 >> shift;
    bmask = 0x0000ff00 >> shift;
    amask = 0x000000ff >> shift;
#else
    rmask = 0x000000ff;
    gmask = 0x0000ff00;
    bmask = 0x00ff0000;
    amask = (icon_image.bytes_per_pixel == (3)) ? 0 : 0xff000000;
#endif
                              
    SDL_Surface* icon = SDL_CreateRGBSurfaceFrom((void*)icon_image.pixel_data, icon_image.width, icon_image.height, 
        icon_image.bytes_per_pixel*8, icon_image.bytes_per_pixel*icon_image.width, rmask, gmask, bmask, amask);
    
    SDL_SetWindowIcon(window, icon);
    SDL_FreeSurface(icon);
    
    //Create OpenGL contexts
    glLoadingContext = SDL_GL_CreateContext(window);
    if(glLoadingContext == NULL)
        cCritical("SDL2: %s", SDL_GetError());
    
    glMainContext = SDL_GL_CreateContext(window);
    if(glMainContext == NULL)
        cCritical("SDL2: %s", SDL_GetError());
    
    //Disable vertical synchronization --> use framerate limitting instead (e.g. max 60 FPS)
    if(SDL_GL_SetSwapInterval(0) == -1)
        cError("SDL2: %s", SDL_GetError());
    
    //Initialize OpenGL function handlers 
    int version = gladLoadGL((GLADloadfunc) SDL_GL_GetProcAddress);
    int vmajor = GLAD_VERSION_MAJOR(version);
    int vminor = GLAD_VERSION_MINOR(version);
    if(vmajor < 4 || (vmajor == 4 && vminor < 3))
        cCritical("This program requires support for OpenGL 4.3, however OpenGL %d.%d was detected! Exiting...", vmajor, vminor);

    //Initialize OpenGL pipeline
    cInfo("Window created. OpenGL %d.%d contexts created.", vmajor, vminor);
    OpenGLState::Init();
    GLSLShader::Init();
    
    //Initialize console output
    std::vector<ConsoleMessage> textLines = console->getLines();
    delete console;
    console = new OpenGLConsole();
    for(size_t i=0; i<textLines.size(); ++i)
        console->AppendMessage(textLines[i]);
    ((OpenGLConsole*)console)->Init(windowW, windowH);
    
    //Create loading thread
    LoadingThreadData* data = new LoadingThreadData();
    data->app = this;
    data->mutex = console->getLinesMutex();
    loadingThread = SDL_CreateThread(GraphicalSimulationApp::RenderLoadingScreen, "loadingThread", data);

    //Look for joysticks
    int jcount = SDL_NumJoysticks();
    
    if(jcount > 0)
    {
        joystick = SDL_JoystickOpen(0);
        joystickButtons = new bool[SDL_JoystickNumButtons(joystick)];
        memset(joystickButtons, 0, SDL_JoystickNumButtons(joystick));
        joystickAxes = new int16_t[SDL_JoystickNumAxes(joystick)];
        memset(joystickAxes, 0, SDL_JoystickNumAxes(joystick) * sizeof(int16_t));
        joystickHats = new uint8_t[SDL_JoystickNumHats(joystick)];
        memset(joystickHats, 0, SDL_JoystickNumHats(joystick));
        cInfo("Joystick %s connected (%d axes, %d hats, %d buttons)", SDL_JoystickName(joystick),
                                                                      SDL_JoystickNumAxes(joystick),
                                                                      SDL_JoystickNumHats(joystick),
                                                                      SDL_JoystickNumButtons(joystick));
    }
}

void GraphicalSimulationApp::InitializeGUI()
{
}

void GraphicalSimulationApp::WindowEvent(SDL_Event* event)
{
    int w, h;
    
    switch(event->window.event)
    {
        case SDL_WINDOWEVENT_RESIZED:
            SDL_GetWindowSize(window, &w, &h);
            gui->Resize(w, h);
            break;
    }
}


void GraphicalSimulationApp::KeyDown(SDL_Event *event)
{
    GLfloat moveStep = 0.1f;
    GLfloat rotateStep = 1.f;
    if(event->key.keysym.mod & KMOD_SHIFT){
        moveStep = 1.f;
        rotateStep = 10.f;
    }
    switch (event->key.keysym.sym)
    {
        case SDLK_ESCAPE:
            Quit();
            break;
            
        case SDLK_SPACE:
             selectedEntity = std::make_pair(nullptr, 0);
             if(!getSimulationManager()->isSimulationFresh())
             {
                 StopSimulation();
                 getSimulationManager()->RestartScenario();
             }
             StartSimulation();
             break;
            
        case SDLK_h:
            displayHUD = !displayHUD;
            break;

        case SDLK_k:
            displayKeymap = !displayKeymap;
            break;

        case SDLK_p:
            displayPerformance = !displayPerformance;
            break;

        case SDLK_c:
            displayConsole = !displayConsole;
            ((OpenGLConsole*)console)->ResetScroll();
            break;
            
        case SDLK_w: //Forward
        {
            OpenGLTrackball* trackball = getSimulationManager()->getTrackball();
            if(trackball->isEnabled())
                trackball->MoveCenter(trackball->GetLookingDirection() * moveStep);
        }
            break;
            
        case SDLK_s: //Backward
        {
            OpenGLTrackball* trackball = getSimulationManager()->getTrackball();
            if(trackball->isEnabled())
                trackball->MoveCenter(-trackball->GetLookingDirection() * moveStep);
        }
            break;
            
        case SDLK_a: //Left
        {
            OpenGLTrackball* trackball = getSimulationManager()->getTrackball();
            if(trackball->isEnabled())
            {
                glm::vec3 axis = glm::cross(trackball->GetLookingDirection(), trackball->GetUpDirection());
                trackball->MoveCenter(-axis * moveStep);
            }
        }
            break;
            
        case SDLK_d: //Right
        {
            OpenGLTrackball* trackball = getSimulationManager()->getTrackball();
            if(trackball->isEnabled())
            {
                glm::vec3 axis = glm::cross(trackball->GetLookingDirection(), trackball->GetUpDirection());
                trackball->MoveCenter(axis * moveStep);
            }
        }
            break;
            
        case SDLK_r: //Up
        {
            OpenGLTrackball* trackball = getSimulationManager()->getTrackball();
            if(trackball->isEnabled())
                trackball->MoveCenter(glm::vec3(0.f, 0.f, -moveStep));
        }
            break;
            
        case SDLK_f: //Down
        {
            OpenGLTrackball* trackball = getSimulationManager()->getTrackball();
            if(trackball->isEnabled())
                trackball->MoveCenter(glm::vec3(0.f, 0.f, moveStep));
        }
            break;
        case SDLK_q: //Rotate left
        {
            OpenGLTrackball* trackball = getSimulationManager()->getTrackball();
            if(trackball->isEnabled())
            {
                glm::quat rotation = glm::angleAxis(glm::radians(-rotateStep), glm::vec3(0.0f, 0.0f, 1.0f));
                trackball->Rotate(rotation);
            }
        }
            break;

        case SDLK_e: // Rotate right.
        {
            OpenGLTrackball* trackball = getSimulationManager()->getTrackball();

            if(trackball->isEnabled())
            {
                glm::quat rotation = glm::angleAxis(glm::radians(rotateStep),glm::vec3(0.0f, 0.0f, 1.0f));
                trackball->Rotate(rotation);
            }
        }
            break;



        default:
            break;
    }
}

void GraphicalSimulationApp::KeyUp(SDL_Event *event)
{
}

void GraphicalSimulationApp::MouseDown(SDL_Event *event)
{
}

void GraphicalSimulationApp::MouseUp(SDL_Event *event)
{
}

void GraphicalSimulationApp::MouseMove(SDL_Event *event)
{
}

void GraphicalSimulationApp::MouseScroll(SDL_Event *event)
{
}

void GraphicalSimulationApp::JoystickDown(SDL_Event *event)
{
}

void GraphicalSimulationApp::JoystickUp(SDL_Event *event)
{
}

void GraphicalSimulationApp::ProcessInputs()
{
}

void GraphicalSimulationApp::LoopInternal()
{
    SDL_Event event;
    SDL_FlushEvents(SDL_FINGERDOWN, SDL_MULTIGESTURE);
            
    while(SDL_PollEvent(&event))
    {
        switch(event.type)
        {
            case SDL_WINDOWEVENT:
                WindowEvent(&event);
                break;
            
            case SDL_TEXTINPUT:
                break;
                
            case SDL_KEYDOWN:
            {
                gui->KeyDown(event.key.keysym.sym);
                KeyDown(&event);
                break;
            }
                
            case SDL_KEYUP:
            {
                gui->KeyUp(event.key.keysym.sym);
                KeyUp(&event);
                break;
            }
                
            case SDL_MOUSEBUTTONDOWN:
            {
                gui->MouseDown(event.button.x, event.button.y, event.button.button == SDL_BUTTON_LEFT);
                mouseWasDown = event;
            }
                break;
                
            case SDL_MOUSEBUTTONUP:
            {
                //GUI
                gui->MouseUp(event.button.x, event.button.y, event.button.button == SDL_BUTTON_LEFT);
                
                //Trackball
                if(event.button.button == SDL_BUTTON_RIGHT || event.button.button == SDL_BUTTON_MIDDLE)
                {
                    OpenGLTrackball* trackball = getSimulationManager()->getTrackball();
                    if(trackball->isEnabled())
                        trackball->MouseUp();
                }
                
                //Pass
                MouseUp(&event);
            }
                break;
                
            case SDL_MOUSEMOTION:
            {
                //GUI
                gui->MouseMove(event.motion.x, event.motion.y);
                
                OpenGLTrackball* trackball = getSimulationManager()->getTrackball();
                if(trackball->isEnabled())
                {
                    GLfloat xPos = (GLfloat)(event.motion.x-getWindowWidth()/2.f)/(GLfloat)(getWindowHeight()/2.f);
                    GLfloat yPos = -(GLfloat)(event.motion.y-getWindowHeight()/2.f)/(GLfloat)(getWindowHeight()/2.f);
                    trackball->MouseMove(xPos, yPos);
                }
                    
                //Pass
                MouseMove(&event);
            }
                break;
                
            case SDL_MOUSEWHEEL:
            {
                if(displayConsole) //GUI
                    ((OpenGLConsole*)console)->Scroll((GLfloat)-event.wheel.y);
                else
                {
                    //Trackball
                    OpenGLTrackball* trackball = getSimulationManager()->getTrackball();
                    if(trackball->isEnabled())
                        trackball->MouseScroll(event.wheel.y * -1.f);
                    
                    //Pass
                    MouseScroll(&event);
                }
            }
                break;
                
            case SDL_JOYBUTTONDOWN:
                joystickButtons[event.jbutton.button] = true;
                JoystickDown(&event);
                break;
                
            case SDL_JOYBUTTONUP:
                joystickButtons[event.jbutton.button] = false;
                JoystickUp(&event);
                break;
                
            case SDL_QUIT:
            {
                if(isRunning())
                    StopSimulation();
                    
                Quit();
            }   
                break;
        }
    }

    if(joystick != NULL)
    {
        for(int i=0; i<SDL_JoystickNumAxes(joystick); i++)
            joystickAxes[i] = SDL_JoystickGetAxis(joystick, i);
    
        for(int i=0; i<SDL_JoystickNumHats(joystick); i++)
            joystickHats[i] = SDL_JoystickGetHat(joystick, i);
    }
    
    ProcessInputs();
    RenderLoop();
    
    //workaround for checking if IMGUI is being manipulated
    if(mouseWasDown.type == SDL_MOUSEBUTTONDOWN && !gui->isAnyActive())
    {
        OpenGLTrackball* trackball = getSimulationManager()->getTrackball();
        
        if(trackball->isEnabled())
        {
            if(mouseWasDown.button.button == SDL_BUTTON_LEFT)
            {
                glm::vec3 eye = trackball->GetEyePosition();
                glm::vec3 ray = trackball->Ray(mouseWasDown.button.x, mouseWasDown.button.y);
                selectedEntity = getSimulationManager()->PickEntity(Vector3(eye.x, eye.y, eye.z), Vector3(ray.x, ray.y, ray.z));
            }
            else //RIGHT OR MIDDLE
            {
                GLfloat xPos = (GLfloat)(mouseWasDown.motion.x-getWindowWidth()/2.f)/(GLfloat)(getWindowHeight()/2.f);
                GLfloat yPos = -(GLfloat)(mouseWasDown.motion.y-getWindowHeight()/2.f)/(GLfloat)(getWindowHeight()/2.f);
                trackball->MouseDown(xPos, yPos, mouseWasDown.button.button == SDL_BUTTON_MIDDLE);
            }
        }
        
        //Pass
        MouseDown(&event);
    }
    mouseWasDown.type = SDL_LASTEVENT;

    //Framerate limitting (60Hz)
    if(limitFramerate)
    {
        uint64_t elapsedTime = GetTimeInMicroseconds() - startTime;
        if(elapsedTime < 16000)
            std::this_thread::sleep_for(std::chrono::microseconds(16000 - elapsedTime));
        startTime = GetTimeInMicroseconds();
    }
}

void GraphicalSimulationApp::RenderLoop()
{
    
    if(!isRunning())
    {
        getSimulationManager()->UpdateDrawingQueue();
    }
    
    size_t sid = 0;
    Sensor* sen;
    std::string t="trackball";
    while((sen = getSimulationManager()->getSensor(sid)) != nullptr)
    {
           ++sid;
           if(sen->getName()==t){
              glm::vec3 eye = getSimulationManager()->getTrackball()->GetEyePosition();
              glm::vec3 dir = getSimulationManager()->getTrackball()->GetLookingDirection();
              glm::vec3 up = getSimulationManager()->getTrackball()->GetUpDirection();
 dynamic_cast<ColorCamera*>(sen)->SetupCamera(Vector3(eye.x,eye.y,eye.z),Vector3(dir.x,dir.y,dir.z),Vector3(up.x,up.y,up.z));
           }
       
    }
    
    //Rendering
    glBeginQuery(GL_TIME_ELAPSED, timeQuery[timeQueryPingpong]);
    glPipeline->Render(getSimulationManager());
    glPipeline->DrawDisplay();
    
    //GUI & Console
    if(displayConsole)
    {
        gui->GenerateBackground();
        ((OpenGLConsole*)console)->Render(true);
    }
    else
    {
        if(displayHUD) //Draw immediate mode GUI
        {
            gui->GenerateBackground();
            gui->Begin();
            DoHUD();
            gui->End();
        }
        else //Just draw logo in the corner
        {
            gui->Begin();
            gui->End();
        }
    }
    glEndQuery(GL_TIME_ELAPSED);
    
    //Update drawing time
    uint64_t drawTime;
    glGetQueryObjectui64v(timeQuery[1-timeQueryPingpong], GL_QUERY_RESULT, &drawTime);
    timeQueryPingpong = 1-timeQueryPingpong;
	double dt = std::min(drawTime/1000000.0, 1000.0); //in ms
	double f = 1.0/60.0;
	drawingTime = f*dt + (1.0-f)*drawingTime;

    //Update maximum drawing time
    if(maxCounter >= 60)
    {
        maxDrawingTime = drawingTime;
        maxCounter = 0;
    }
    else
    {
        maxDrawingTime = std::max(maxDrawingTime, dt);
        ++maxCounter;
    }

    /*glPixelStorei(GL_PACK_ALIGNMENT, 1); // Set pixel alignment
    glReadPixels(0, 0, 400,400, GL_RGB, GL_UNSIGNED_BYTE, pixels.data());
    stbi_flip_vertically_on_write(true); // Ensure correct orientation
    stbi_write_png(("/home/michele/image_stonefish/output_stone_" + std::to_string(turbidity_level) + ".png").c_str(), 400, 400, 3, pixels.data(), 0);

    turbidity_level++;
    */

    

    //glFinish(); //Ensure that the frame was fully rendered
    SDL_GL_SwapWindow(window);
}



void GraphicalSimulationApp::DoHUD()
{
    char buf[256];
    
    //Helper settings
    HelperSettings& hs = getHelperSettings();
    Ocean* ocn = getSimulationManager()->getOcean();
    if(getTurbidityLevel()!=-1){
       ocn->setJerlovUse(false);
    }
    
    GLfloat offset = 10.f;
    gui->DoPanel(10.f, offset, 160.f, ocn != NULL ? 250.f : 259.f);
    offset += 5.f;
    gui->DoLabel(15.f, offset, "DEBUG");
    offset += 15.f;
    
    Uid id;
    id.owner = 0;
    
    id.item = 0;
    bool displayPhysical = getSimulationManager()->getSolidDisplayMode() == DisplayMode::PHYSICAL; 
    displayPhysical = gui->DoCheckBox(id, 15.f, offset, 110.f, displayPhysical, "Physical objects");
    getSimulationManager()->setSolidDisplayMode(displayPhysical ? DisplayMode::PHYSICAL : DisplayMode::GRAPHICAL);
    offset += 22.f;

    id.item = 1;
    hs.showCoordSys = gui->DoCheckBox(id, 15.f, offset, 110.f, hs.showCoordSys, "Frames");
    offset += 22.f;
    
    id.item = 2;
    hs.showSensors = gui->DoCheckBox(id, 15.f, offset, 110.f, hs.showSensors, "Sensors");
    offset += 22.f;
    
    id.item = 3;
    hs.showActuators = gui->DoCheckBox(id, 15.f, offset, 110.f, hs.showActuators, "Actuators");
    offset += 22.f;
    
    id.item = 4;
    hs.showJoints = gui->DoCheckBox(id, 15.f, offset, 110.f, hs.showJoints, "Joints");
    offset += 22.f;
    
    id.item = 5;
    hs.showBulletDebugInfo = gui->DoCheckBox(id, 15.f, offset, 110.f, hs.showBulletDebugInfo, "Collision");
    offset += 22.f;
    
    if(ocn != nullptr)
    {
        id.item = 6;
        hs.showForces = gui->DoCheckBox(id, 15.f, offset, 110.f, hs.showForces, "Fluid Forces");
        offset += 22.f;
    
        id.item = 7;
        hs.showFluidDynamics = gui->DoCheckBox(id, 15.f, offset, 110.f, hs.showFluidDynamics, "Hydrodynamics");
        offset += 22.f;

        id.item = 8;
        hs.showOceanVelocityField = gui->DoCheckBox(id, 15.f, offset, 110.f, hs.showOceanVelocityField, "Water velocity");
        offset += 22.f;
    }
    

    id.item=9;
    bool annotation = gui->DoCheckBox(id, 15.f, offset, 110.f, getSimulationManager()->isAnnotationEnabled(), "Automatic Annotation");
    getSimulationManager()->setAnnotation(annotation);
    
    
    
    offset += 22.f;
    
    offset += 14.f;
    
    //Time settings
    Scalar az, elev;
    getSimulationManager()->getAtmosphere()->GetSunPosition(az, elev);
    
    gui->DoPanel(10.f, offset, 160.f, 125.f);
    offset += 5.f;
    gui->DoLabel(15.f, offset, "SUN POSITION");
    offset += 15.f;
    
    id.owner = 1;
    id.item = 0;
    az = gui->DoSlider(id, 15.f, offset, 150.f, Scalar(-180), Scalar(180), az, "Azimuth[deg]");
    offset += 50.f;
    
    id.item = 1;
    elev = gui->DoSlider(id, 15.f, offset, 150.f, Scalar(-10), Scalar(90), elev, "Elevation[deg]");
    offset += 61.f;
    
    getSimulationManager()->getAtmosphere()->SetupSunPosition(az, elev);
    
    //Ocean settings
    if(ocn != nullptr)
    {
        Scalar waterType = ocn->getWaterType();
        //Scalar scattering = ocn->getSfactor();
        
        bool oceanOn = ocn->isRenderable();
        
        gui->DoPanel(10.f, offset, 160.f, oceanOn ? 112.f : 33.f);
        offset += 5.f;
       
        id.owner = 2;
        id.item = 0;
        ocn->setRenderable(gui->DoCheckBox(id, 15.f, offset, 110.f, oceanOn, "OCEAN"));
        offset += 26.f;
        
        if(oceanOn)
        {
            id.item = 1;
            waterType = gui->DoSlider(id, 15.f, offset, 150.f, Scalar(0), Scalar(1), waterType, "Jerlov water type");
            ocn->setWaterType(waterType);
            offset += 50.f;
            /*id.item = 3;
            scattering = gui->DoSlider(id, 15.f, offset, 150.f, Scalar(0), Scalar(1000), scattering, "Scattering coeff.");
            ocn->setSfactor(scattering);
            offset += 40.f;
            */
            id.item = 2;
            ocn->setParticles(gui->DoCheckBox(id, 19.f, offset, 110.f, ocn->hasParticles(), "Suspended particles"));
            offset += 29.f;
        }

        offset += 8.f;
    }
    
    
    //Main view exposure
    gui->DoPanel(10.f, offset, 160.f, 126.f);
    offset += 5.f;
    gui->DoLabel(15.f, offset, "VIEW");
    offset += 15.f;
    
    id.owner = 3;
    id.item = 0;
    std::vector<std::string> options;
    options.push_back("Free");

    unsigned int selected = 0;
    unsigned int newSelected = 0;

    //Add robots to the list
    size_t rid = 0;
    Robot* rob;
    while((rob = getSimulationManager()->getRobot(rid)) != nullptr)
    {
        options.push_back(rob->getName());
        if(rob->getBaseLink() == trackballCenter)
            selected = (unsigned int)(rid + 1);
        ++rid;
    }
    //Add animated entities to the list
    size_t eid = 0;
    size_t aid = 0;
    Entity* ent;
    while((ent = getSimulationManager()->getEntity(eid)) != nullptr)
    {
        if(ent->getType() == sf::EntityType::ANIMATED)
        {
            options.push_back(ent->getName());
            if(ent == trackballCenter)
                selected = (unsigned int)(rid + 1 + aid);
            ++aid;
        }
        ++eid;
    }

    newSelected = gui->DoComboBox(id, 15.f, offset, 150.f, options, selected, "Trackball center");
    
    if(newSelected != selected)
    {
        if(newSelected == 0)
            trackballCenter = nullptr;
        else
        {
            if(newSelected <= rid)
                trackballCenter = getSimulationManager()->getRobot(options[newSelected])->getBaseLink();
            else if(newSelected > rid)
                trackballCenter = (MovingEntity*)getSimulationManager()->getEntity(options[newSelected]);
        }     
        getSimulationManager()->getTrackball()->GlueToMoving(trackballCenter);
    }
    offset += 51.f;
    
    id.owner = 3;
    id.item = 1;
    getSimulationManager()->getTrackball()->setExposureCompensation(gui->DoSlider(id, 15.f, offset, 150.f, Scalar(-3), Scalar(3), getSimulationManager()->getTrackball()->getExposureCompensation(), "Exposure[EV]"));
    offset += 61.f;
    
    //Picked entity information
    if(selectedEntity.first != nullptr)
    {
        switch(selectedEntity.first->getType())
        {
            case EntityType:: STATIC:
            {
                StaticEntity* ent = (StaticEntity*)selectedEntity.first;
                
                gui->DoPanel(180.f, offset, 160.f, 66.f);
                offset += 5.f;
                gui->DoLabel(195.f, offset, "SELECTION INFO");
                offset += 16.f;
                gui->DoLabel(198.f, offset, std::string("Name: ") + ent->getName());
                offset += 14.f;
                gui->DoLabel(198.f, offset, std::string("Type: Static"));
                offset += 14.f;
                gui->DoLabel(198.f, offset, std::string("Material: ") + ent->getMaterial().name);
                offset -= 49.f;
            }
                break;
                
            case EntityType:: SOLID:
            {
                SolidEntity* ent = (SolidEntity*)selectedEntity.first;


                std::vector<Vector3>* x = ent->getMeshVertices();
		// ANNOTATION CAMERA
		glm::mat4 projectionMatrix = getSimulationManager()->getTrackball()->GetProjectionMatrix();
		glm::mat4 viewMatrix = getSimulationManager()->getTrackball()->GetViewMatrix();

		// Compute the combined view-projection matrix
		glm::mat4 viewProjectionMatrix = projectionMatrix * viewMatrix;

		// Get the minimum and maximum points of the bounding box from the SolidEntity
		Vector3 minPoint, maxPoint;
		ent->getAABB(minPoint, maxPoint);

		// Get the position and orientation of the object
		Vector3 xyz = ent->getCGTransform().getOrigin();
		btQuaternion q;
		ent->getCGTransform().getBasis().getRotation(q);
		Vector3 rpy = Vector3(q.x(), q.y(), q.z());

		// Construct the transformation matrix
		glm::mat4 modelMatrix = glm::translate(glm::mat4(1.0f), glm::vec3(0, 0,0)) *
				        glm::mat4(glm::yawPitchRoll(glm::radians(rpy[1]), glm::radians(rpy[0]), glm::radians(rpy[2])));

		// Transform the bounding box corners into world space
		glm::vec4 corners[8];
		corners[0] = modelMatrix * glm::vec4(minPoint[0], minPoint[1], minPoint[2], 1.0f);
		corners[1] = modelMatrix * glm::vec4(maxPoint[0], minPoint[1], minPoint[2], 1.0f);
		corners[2] = modelMatrix * glm::vec4(maxPoint[0], maxPoint[1], minPoint[2], 1.0f);
		corners[3] = modelMatrix * glm::vec4(minPoint[0], maxPoint[1], minPoint[2], 1.0f);
		corners[4] = modelMatrix * glm::vec4(minPoint[0], minPoint[1], maxPoint[2], 1.0f);
		corners[5] = modelMatrix * glm::vec4(maxPoint[0], minPoint[1], maxPoint[2], 1.0f);
		corners[6] = modelMatrix * glm::vec4(maxPoint[0], maxPoint[1], maxPoint[2], 1.0f);
		corners[7] = modelMatrix * glm::vec4(minPoint[0], maxPoint[1], maxPoint[2], 1.0f);

		// Convert world space coordinates to clip space
		glm::vec4 cornersClip[8];
		for (int i = 0; i < 8; ++i) {
		    cornersClip[i] = viewProjectionMatrix * corners[i];
		    cornersClip[i] /= cornersClip[i].w; // Convert to normalized device coordinates
		}

		// Convert clip space coordinates to screen space
		float screenWidth = static_cast<float>(1200);
		float screenHeight = static_cast<float>(800);

		glm::vec2 minPointScreen = glm::vec2(std::numeric_limits<float>::max());
		glm::vec2 maxPointScreen = glm::vec2(std::numeric_limits<float>::min());

		for (int i = 0; i < 8; ++i) {
		    minPointScreen.x = std::min(minPointScreen.x, (cornersClip[i].x + 1.0f) * 0.5f * screenWidth);
		    minPointScreen.y = std::min(minPointScreen.y, (1.0f - cornersClip[i].y) * 0.5f * screenHeight);
		    maxPointScreen.x = std::max(maxPointScreen.x, (cornersClip[i].x + 1.0f) * 0.5f * screenWidth);
		    maxPointScreen.y = std::max(maxPointScreen.y, (1.0f - cornersClip[i].y) * 0.5f * screenHeight);
		}

		// Add a small margin to ensure the bounding box covers the entire object
		const float margin = 5.0f;
		minPointScreen -= glm::vec2(margin, margin);
		maxPointScreen += glm::vec2(margin, margin);

		// Draw rounded rectangle using the adjusted screen space coordinates
		gui->DrawRoundedRect(minPointScreen.x, minPointScreen.y,
				     maxPointScreen.x - minPointScreen.x,
				     maxPointScreen.y - minPointScreen.y,
				     glm::vec4(0.0f, 0.0f, 0.0f, 0.1f));


                float compensation=0;
                GLfloat infoOffset = offset;
                gui->DoPanel(180.f, offset, 160.f, ent->getSolidType() == SolidType::COMPOUND ? 130.f : 122.f);
                offset += 5.f;
                compensation+=5.f;
                gui->DoLabel(195.f, offset, "SELECTION INFO");
                offset += 16.f;
                compensation+=16.f;
                gui->DoLabel(198.f, offset, std::string("Name: ") + ent->getName());
                offset += 14.f;
                compensation+=14.f;
                gui->DoLabel(198.f, offset, std::string("Type: Dynamic"));
                offset += 14.f;
                compensation+=14.f;
                if(ent->getSolidType() != SolidType::COMPOUND)
                {
                    gui->DoLabel(198.f, offset, std::string("Material: ") + ent->getMaterial().name);
                    offset += 14.f;
                }
                std::sprintf(buf, "%1.3lf", ent->getMass());
                gui->DoLabel(198.f, offset, std::string("Mass[kg]: ") + std::string(buf));
                offset += 14.f;
                compensation+=14.f;
                gui->DoLabel(198.f, offset, std::string("Inertia[kgm2]: "));
                offset += 14.f;
                compensation+=14.f;
                Vector3 I = ent->getInertia();
                std::sprintf(buf, "%1.3lf, %1.3lf, %1.3lf", I.x(), I.y(), I.z());
                gui->DoLabel(203.f, offset, std::string(buf));
                offset += 14.f;
                std::sprintf(buf, "%1.3lf", ent->getVolume()*1e3);
                gui->DoLabel(198.f, offset, std::string("Volume[dm3]: ") + std::string(buf));
                offset += 11.f;
                compensation+=2.f;
                
                if(ent->getSolidType() == SolidType::COMPOUND)
                {
                    Compound* cmp = (Compound*)ent;
                    id.owner = 4;
                    id.item = 0;
                    cmp->setDisplayInternalParts(gui->DoCheckBox(id, 215.f, offset, 110.f, cmp->isDisplayingInternalParts(), "Show internals"));
                    offset += 22.f;
                    compensation+=22.f;

                    CompoundPart part = cmp->getPart(cmp->getPartId(selectedEntity.second));
                    if(part.solid != nullptr)
                    {
                        offset = infoOffset + 10.f;
                        GLfloat hOffset = 165.f;
                        gui->DoPanel(hOffset + 190.f, offset, 130.f, 110.f);
                        offset += 5.f;
                        gui->DoLabel(hOffset + 195.f, offset, "PART INFO");
                        offset += 16.f;
                        std::string partName = part.solid->getName();
                        int beginIdx = partName.rfind('/');
                        gui->DoLabel(hOffset + 198.f, offset, std::string("Name: ") + partName.substr(beginIdx + 1));
                        offset += 14.f;
                        gui->DoLabel(hOffset + 198.f, offset, std::string("Material: ") + part.solid->getMaterial().name);
                        offset += 14.f;
                        std::sprintf(buf, "%1.3lf", part.solid->getMass());
                        gui->DoLabel(hOffset + 198.f, offset, std::string("Mass[kg]: ") + std::string(buf));
                        offset += 14.f;
                        gui->DoLabel(hOffset + 198.f, offset, std::string("Inertia[kgm2]: "));
                        offset += 14.f;
                        Vector3 I = part.solid->getInertia();
                        std::sprintf(buf, "%1.3lf, %1.3lf, %1.3lf", I.x(), I.y(), I.z());
                        gui->DoLabel(hOffset + 203.f, offset, std::string(buf));
                        offset += 14.f;
                        std::sprintf(buf, "%1.3lf", part.solid->getVolume()*1e3);
                        gui->DoLabel(hOffset + 198.f, offset, std::string("Volume[dm3]: ") + std::string(buf));
                    }
                }
                offset-=compensation;
            }
                break;
                
            default:
                break;
        }
    }
    
    
    
    if(annotation){
	    Entity* ent;
	    SolidEntity* solid_ent;
	    
	    size_t eid = 0;
            std::time_t currentTime = std::time(nullptr); 
            // Convert timestamp to string
            std::stringstream ss;
            ss << std::put_time(std::localtime(&currentTime), "%Y-%m-%d_%H-%M-%S");
            std::string timestampStr = ss.str();
            // Construct filename with timestamp and day
            std::string folderName = "annotations_" + timestampStr.substr(0, 10); // Extract YYYY-MM-DD
            std::string filename = "annotations_" + timestampStr + ".txt";
            std::string filename_seg = "segmentation_" + timestampStr + ".png";
            // Create directory if it doesn't exist
            bool create_directory=false;
            #ifdef _WIN32
            int status = _mkdir(folderName.c_str());
            #else
            int status = mkdir(folderName.c_str(), 0777); // 0777 gives read, write, execute permission to everyone
            #endif
            // Construct full path for filename
            std::string fullPath = folderName + "/" + filename;
            std::string fullPathSeg = folderName+"/"+filename_seg;
            // Open a file for writing annotations
            std::ofstream annotationFile(fullPath);
            if (!annotationFile.is_open()) {
                std::cerr << "Error: Failed to open file for writing." << std::endl;
            }
            // Convert clip space coordinates to screen space
            float screenWidth = static_cast<float>(1200);
            float screenHeight = static_cast<float>(800);
            cv::Mat segmentationMask = cv::Mat::zeros(screenHeight, screenWidth, CV_8UC3);
	    while((ent = getSimulationManager()->getEntity(eid)) != nullptr)
	    {
		++eid;
		if(ent->getType()==EntityType::SOLID || ent->getType()==EntityType::STATIC ){
		SolidEntity* solid_ent;
		StaticEntity* static_ent;
		if(ent->getType()==EntityType::SOLID){
		   solid_ent = (SolidEntity*)ent; 
		     std::vector<Vector3>* x = solid_ent->getMeshVertices();
		// ANNOTATION CAMERA
		glm::mat4 projectionMatrix = getSimulationManager()->getTrackball()->GetProjectionMatrix();
		glm::mat4 viewMatrix = getSimulationManager()->getTrackball()->GetViewMatrix();

		// Compute the combined view-projection matrix
		glm::mat4 viewProjectionMatrix = projectionMatrix * viewMatrix;

		// Get the minimum and maximum points of the bounding box from the SolidEntity
		Vector3 minPoint, maxPoint;
		solid_ent->getAABB(minPoint, maxPoint);

		// Get the position and orientation of the object
		Vector3 xyz = solid_ent->getCGTransform().getOrigin();
		btQuaternion q;
		solid_ent->getCGTransform().getBasis().getRotation(q);
		Vector3 rpy = Vector3(q.x(), q.y(), q.z());

		// Construct the transformation matrix
		glm::mat4 modelMatrix = glm::translate(glm::mat4(1.0f), glm::vec3(0, 0,0)) *
				        glm::mat4(glm::yawPitchRoll(glm::radians(rpy[1]), glm::radians(rpy[0]), glm::radians(rpy[2])));

		// Transform the bounding box corners into world space
		glm::vec4 corners[8];
		corners[0] = modelMatrix * glm::vec4(minPoint[0], minPoint[1], minPoint[2], 1.0f);
		corners[1] = modelMatrix * glm::vec4(maxPoint[0], minPoint[1], minPoint[2], 1.0f);
		corners[2] = modelMatrix * glm::vec4(maxPoint[0], maxPoint[1], minPoint[2], 1.0f);
		corners[3] = modelMatrix * glm::vec4(minPoint[0], maxPoint[1], minPoint[2], 1.0f);
		corners[4] = modelMatrix * glm::vec4(minPoint[0], minPoint[1], maxPoint[2], 1.0f);
		corners[5] = modelMatrix * glm::vec4(maxPoint[0], minPoint[1], maxPoint[2], 1.0f);
		corners[6] = modelMatrix * glm::vec4(maxPoint[0], maxPoint[1], maxPoint[2], 1.0f);
		corners[7] = modelMatrix * glm::vec4(minPoint[0], maxPoint[1], maxPoint[2], 1.0f);

		// Convert world space coordinates to clip space
		glm::vec4 cornersClip[8];
		for (int i = 0; i < 8; ++i) {
		    cornersClip[i] = viewProjectionMatrix * corners[i];
		    cornersClip[i] /= cornersClip[i].w; // Convert to normalized device coordinates
		}
		
    // Check if any corner of the bounding box is within the screen boundaries
    bool isVisible = false;
    for (int i = 0; i < 8; ++i) {
        // Check if the corner is within the screen boundaries in NDC
        if (cornersClip[i].x >= -1.0f && cornersClip[i].x <= 1.0f && cornersClip[i].y >= -1.0f && cornersClip[i].y <= 1.0f) {
            isVisible = true;
            break;
        }
    }

		glm::vec2 minPointScreen = glm::vec2(std::numeric_limits<float>::max());
		glm::vec2 maxPointScreen = glm::vec2(std::numeric_limits<float>::min());

		for (int i = 0; i < 8; ++i) {
		    minPointScreen.x = std::min(minPointScreen.x, (cornersClip[i].x + 1.0f) * 0.5f * screenWidth);
		    minPointScreen.y = std::min(minPointScreen.y, (1.0f - cornersClip[i].y) * 0.5f * screenHeight);
		    maxPointScreen.x = std::max(maxPointScreen.x, (cornersClip[i].x + 1.0f) * 0.5f * screenWidth);
		    maxPointScreen.y = std::max(maxPointScreen.y, (1.0f - cornersClip[i].y) * 0.5f * screenHeight);
		}

		// Add a small margin to ensure the bounding box covers the entire object
		const float margin = 5.0f;
		minPointScreen -= glm::vec2(margin, margin);
		maxPointScreen += glm::vec2(margin, margin);

		// Draw rounded rectangle using the adjusted screen space coordinates
		gui->DrawRoundedRect(minPointScreen.x, minPointScreen.y,
				     maxPointScreen.x - minPointScreen.x,
				     maxPointScreen.y - minPointScreen.y,
				     glm::vec4(0.0f, 0.0f, 0.0f, 0.1f));
				     
		if(isVisible){
		float xmin = minPointScreen.x;
		float ymin = minPointScreen.y;
		float xmax = maxPointScreen.x;
		float ymax = maxPointScreen.y;
		annotationFile << ent->getName()  << " " << xmin << " " << ymin << " " << xmax << " " << ymax << "\n";
		}
		
if (isVisible) {
    // Iterate over each vertex in the mesh, assuming consecutive triangles
    for (size_t i = 0; i < x->size(); i += 3) {
        // Get the vertices of the triangle
        const auto& v1 = (*x)[i];
        const auto& v2 = (*x)[i + 1];
        const auto& v3 = (*x)[i + 2];

        // Apply model-view-projection transformation to the vertices
        glm::vec4 clip1 = viewProjectionMatrix * modelMatrix * glm::vec4(v1[0], v1[1], v1[2], 1.0f);
        glm::vec4 clip2 = viewProjectionMatrix * modelMatrix * glm::vec4(v2[0], v2[1], v2[2], 1.0f);
        glm::vec4 clip3 = viewProjectionMatrix * modelMatrix * glm::vec4(v3[0], v3[1], v3[2], 1.0f);

        // Convert to normalized device coordinates
        clip1 /= clip1.w;
        clip2 /= clip2.w;
        clip3 /= clip3.w;

        // Check if the triangle is within the screen boundaries
        if (clip1.x >= -1.0f && clip1.x <= 1.0f && clip1.y >= -1.0f && clip1.y <= 1.0f &&
            clip2.x >= -1.0f && clip2.x <= 1.0f && clip2.y >= -1.0f && clip2.y <= 1.0f &&
            clip3.x >= -1.0f && clip3.x <= 1.0f && clip3.y >= -1.0f && clip3.y <= 1.0f) {
            // Convert normalized device coordinates to screen space
            glm::vec2 screen1 = glm::vec2((clip1.x + 1.0f) * 0.5f * screenWidth, (1.0f - clip1.y) * 0.5f * screenHeight);
            glm::vec2 screen2 = glm::vec2((clip2.x + 1.0f) * 0.5f * screenWidth, (1.0f - clip2.y) * 0.5f * screenHeight);
            glm::vec2 screen3 = glm::vec2((clip3.x + 1.0f) * 0.5f * screenWidth, (1.0f - clip3.y) * 0.5f * screenHeight);

            // Find the bounding box of the triangle
            int minX = std::min(std::min(screen1.x, screen2.x), screen3.x);
            int maxX = std::max(std::max(screen1.x, screen2.x), screen3.x);
            int minY = std::min(std::min(screen1.y, screen2.y), screen3.y);
            int maxY = std::max(std::max(screen1.y, screen2.y), screen3.y);

            // Iterate over each pixel in the bounding box
            for (int x = minX; x <= maxX; ++x) {
                for (int y = minY; y <= maxY; ++y) {
                    // Calculate barycentric coordinates
                    float alpha = ((screen2.y - screen3.y) * (x - screen3.x) + (screen3.x - screen2.x) * (y - screen3.y)) /
                                    ((screen2.y - screen3.y) * (screen1.x - screen3.x) + (screen3.x - screen2.x) * (screen1.y - screen3.y));
                    float beta = ((screen3.y - screen1.y) * (x - screen3.x) + (screen1.x - screen3.x) * (y - screen3.y)) /
                                    ((screen2.y - screen3.y) * (screen1.x - screen3.x) + (screen3.x - screen2.x) * (screen1.y - screen3.y));
                    float gamma = 1.0f - alpha - beta;

                    // If the pixel is inside the triangle
                    if (alpha >= 0 && beta >= 0 && gamma >= 0) {
                        // Fill the pixel in the segmentation mask with the specified color
                        segmentationMask.at<cv::Vec3b>(y, x) = generateColor(eid); // Fill with blue color
                    }
                }
            }
        }
    }
}
		}
		else{
		   static_ent=(StaticEntity*)ent; 
		     std::vector<Vector3>* x = static_ent->getMeshVertices();
		// ANNOTATION CAMERA
		glm::mat4 projectionMatrix = getSimulationManager()->getTrackball()->GetProjectionMatrix();
		glm::mat4 viewMatrix = getSimulationManager()->getTrackball()->GetViewMatrix();

		// Compute the combined view-projection matrix
		glm::mat4 viewProjectionMatrix = projectionMatrix * viewMatrix;

		// Get the minimum and maximum points of the bounding box from the SolidEntity
		Vector3 minPoint, maxPoint;
		static_ent->getAABB(minPoint, maxPoint);

		// Get the position and orientation of the object
		Vector3 xyz = static_ent->getTransform().getOrigin();
		btQuaternion q;
		static_ent->getTransform().getBasis().getRotation(q);
		Vector3 rpy = Vector3(q.x(), q.y(), q.z());

		// Construct the transformation matrix
		glm::mat4 modelMatrix = glm::translate(glm::mat4(1.0f), glm::vec3(0, 0,0)) *
				        glm::mat4(glm::yawPitchRoll(glm::radians(rpy[1]), glm::radians(rpy[0]), glm::radians(rpy[2])));

		// Transform the bounding box corners into world space
		glm::vec4 corners[8];
		corners[0] = modelMatrix * glm::vec4(minPoint[0], minPoint[1], minPoint[2], 1.0f);
		corners[1] = modelMatrix * glm::vec4(maxPoint[0], minPoint[1], minPoint[2], 1.0f);
		corners[2] = modelMatrix * glm::vec4(maxPoint[0], maxPoint[1], minPoint[2], 1.0f);
		corners[3] = modelMatrix * glm::vec4(minPoint[0], maxPoint[1], minPoint[2], 1.0f);
		corners[4] = modelMatrix * glm::vec4(minPoint[0], minPoint[1], maxPoint[2], 1.0f);
		corners[5] = modelMatrix * glm::vec4(maxPoint[0], minPoint[1], maxPoint[2], 1.0f);
		corners[6] = modelMatrix * glm::vec4(maxPoint[0], maxPoint[1], maxPoint[2], 1.0f);
		corners[7] = modelMatrix * glm::vec4(minPoint[0], maxPoint[1], maxPoint[2], 1.0f);

		// Convert world space coordinates to clip space
		glm::vec4 cornersClip[8];
		for (int i = 0; i < 8; ++i) {
		    cornersClip[i] = viewProjectionMatrix * corners[i];
		    cornersClip[i] /= cornersClip[i].w; // Convert to normalized device coordinates
		}
		
    // Check if any corner of the bounding box is within the screen boundaries
    bool isVisible = false;
    for (int i = 0; i < 8; ++i) {
        // Check if the corner is within the screen boundaries in NDC
        if (cornersClip[i].x >= -1.0f && cornersClip[i].x <= 1.0f && cornersClip[i].y >= -1.0f && cornersClip[i].y <= 1.0f) {
            isVisible = true;
            break;
        }
    }

		glm::vec2 minPointScreen = glm::vec2(std::numeric_limits<float>::max());
		glm::vec2 maxPointScreen = glm::vec2(std::numeric_limits<float>::min());

		for (int i = 0; i < 8; ++i) {
		    minPointScreen.x = std::min(minPointScreen.x, (cornersClip[i].x + 1.0f) * 0.5f * screenWidth);
		    minPointScreen.y = std::min(minPointScreen.y, (1.0f - cornersClip[i].y) * 0.5f * screenHeight);
		    maxPointScreen.x = std::max(maxPointScreen.x, (cornersClip[i].x + 1.0f) * 0.5f * screenWidth);
		    maxPointScreen.y = std::max(maxPointScreen.y, (1.0f - cornersClip[i].y) * 0.5f * screenHeight);
		}

		// Add a small margin to ensure the bounding box covers the entire object
		const float margin = 5.0f;
		minPointScreen -= glm::vec2(margin, margin);
		maxPointScreen += glm::vec2(margin, margin);

		// Draw rounded rectangle using the adjusted screen space coordinates
		gui->DrawRoundedRect(minPointScreen.x, minPointScreen.y,
				     maxPointScreen.x - minPointScreen.x,
				     maxPointScreen.y - minPointScreen.y,
				     glm::vec4(0.0f, 0.0f, 0.0f, 0.1f));
				     
		if(isVisible){
		float xmin = minPointScreen.x;
		float ymin = minPointScreen.y;
		float xmax = maxPointScreen.x;
		float ymax = maxPointScreen.y;
		annotationFile << ent->getName()  << " " << xmin << " " << ymin << " " << xmax << " " << ymax << "\n";
		}
		
// Iterate over each vertex in the mesh, assuming consecutive triangles
if(isVisible){
for (size_t i = 0; i < x->size(); i += 3) {
    // Get the vertices of the triangle
    const auto& v1 = (*x)[i];
    const auto& v2 = (*x)[i + 1];
    const auto& v3 = (*x)[i + 2];

    // Apply model-view-projection transformation to the vertices
    glm::vec4 clip1 = viewProjectionMatrix * modelMatrix * glm::vec4(v1[0], v1[1], v1[2], 1.0f);
    glm::vec4 clip2 = viewProjectionMatrix * modelMatrix * glm::vec4(v2[0], v2[1], v2[2], 1.0f);
    glm::vec4 clip3 = viewProjectionMatrix * modelMatrix * glm::vec4(v3[0], v3[1], v3[2], 1.0f);

    // Convert to normalized device coordinates
    clip1 /= clip1.w;
    clip2 /= clip2.w;
    clip3 /= clip3.w;

    // Check if the triangle is within the screen boundaries
    if (clip1.x >= -1.0f && clip1.x <= 1.0f && clip1.y >= -1.0f && clip1.y <= 1.0f &&
        clip2.x >= -1.0f && clip2.x <= 1.0f && clip2.y >= -1.0f && clip2.y <= 1.0f &&
        clip3.x >= -1.0f && clip3.x <= 1.0f && clip3.y >= -1.0f && clip3.y <= 1.0f) {
        // Convert normalized device coordinates to screen space
        glm::vec2 screen1 = glm::vec2((clip1.x + 1.0f) * 0.5f * screenWidth, (1.0f - clip1.y) * 0.5f * screenHeight);
        glm::vec2 screen2 = glm::vec2((clip2.x + 1.0f) * 0.5f * screenWidth, (1.0f - clip2.y) * 0.5f * screenHeight);
        glm::vec2 screen3 = glm::vec2((clip3.x + 1.0f) * 0.5f * screenWidth, (1.0f - clip3.y) * 0.5f * screenHeight);

        // Find the bounding box of the triangle
        int minX = std::min(std::min(screen1.x, screen2.x), screen3.x);
        int maxX = std::max(std::max(screen1.x, screen2.x), screen3.x);
        int minY = std::min(std::min(screen1.y, screen2.y), screen3.y);
        int maxY = std::max(std::max(screen1.y, screen2.y), screen3.y);

        // Clip bounding box to object boundaries
        minX = std::max(minX, 0);
        maxX = std::min(maxX, static_cast<int>(screenWidth) - 1);
        minY = std::max(minY, 0);
        maxY = std::min(maxY, static_cast<int>(screenHeight) - 1);

        // Iterate over each pixel in the clipped bounding box
        for (int x = minX; x <= maxX; ++x) {
            for (int y = minY; y <= maxY; ++y) {
                // Calculate barycentric coordinates
                float alpha = ((screen2.y - screen3.y) * (x - screen3.x) + (screen3.x - screen2.x) * (y - screen3.y)) /
                                ((screen2.y - screen3.y) * (screen1.x - screen3.x) + (screen3.x - screen2.x) * (screen1.y - screen3.y));
                float beta = ((screen3.y - screen1.y) * (x - screen3.x) + (screen1.x - screen3.x) * (y - screen3.y)) /
                                ((screen2.y - screen3.y) * (screen1.x - screen3.x) + (screen3.x - screen2.x) * (screen1.y - screen3.y));
                float gamma = 1.0f - alpha - beta;

                // If the pixel is inside the triangle
                if (alpha >= 0 && beta >= 0 && gamma >= 0) {
                    // Fill the pixel in the segmentation mask with the specified color
                    segmentationMask.at<cv::Vec3b>(y, x) = generateColor(eid); // Fill with blue color
                }
            }
        }
    }
}
}

		   
		   }
              
		
			}
		
		}
    size_t rid = 0;
    Robot* rob;

    currentTime = std::time(nullptr);
    ss;
    ss << std::put_time(std::localtime(&currentTime), "%Y-%m-%d_%H-%M-%S");
    timestampStr = ss.str();
    folderName = "annotations_" + timestampStr.substr(0, 10);

    #ifdef _WIN32
    status = _mkdir(folderName.c_str());
    #else
    status = mkdir(folderName.c_str(), 0777);
    #endif

    std::string filePath = folderName + "/annotations_" + timestampStr + ".txt";
    annotationFile.open(filePath);
    if (!annotationFile.is_open()) {
        std::cerr << "Error: Failed to open file for writing." << std::endl;
    }

  cv::Mat semanticSegmentationMask = cv::Mat::zeros(screenHeight, screenWidth, CV_8UC3);
    cv::Mat panopticSegmentationMask = cv::Mat::zeros(screenHeight, screenWidth, CV_8UC3);

    while((rob = getSimulationManager()->getRobot(rid)) != nullptr) {
        ++rid;
        solid_ent = rob->getBaseLink();
        std::vector<Vector3>* x = solid_ent->getMeshVertices();

        glm::mat4 projectionMatrix = getSimulationManager()->getTrackball()->GetProjectionMatrix();
        glm::mat4 viewMatrix = getSimulationManager()->getTrackball()->GetViewMatrix();
        glm::mat4 viewProjectionMatrix = projectionMatrix * viewMatrix;

        Vector3 minPoint, maxPoint;
        solid_ent->getAABB(minPoint, maxPoint);

        Vector3 xyz = solid_ent->getCGTransform().getOrigin();
        btQuaternion q;
        solid_ent->getCGTransform().getBasis().getRotation(q);
        Vector3 rpy = Vector3(q.x(), q.y(), q.z());

        glm::mat4 modelMatrix = glm::translate(glm::mat4(1.0f), glm::vec3(0, 0, 0)) *
                                glm::mat4(glm::yawPitchRoll(glm::radians(rpy[1]), glm::radians(rpy[0]), glm::radians(rpy[2])));

        glm::vec4 corners[8];
        corners[0] = modelMatrix * glm::vec4(minPoint[0], minPoint[1], minPoint[2], 1.0f);
        corners[1] = modelMatrix * glm::vec4(maxPoint[0], minPoint[1], minPoint[2], 1.0f);
        corners[2] = modelMatrix * glm::vec4(maxPoint[0], maxPoint[1], minPoint[2], 1.0f);
        corners[3] = modelMatrix * glm::vec4(minPoint[0], maxPoint[1], minPoint[2], 1.0f);
        corners[4] = modelMatrix * glm::vec4(minPoint[0], minPoint[1], maxPoint[2], 1.0f);
        corners[5] = modelMatrix * glm::vec4(maxPoint[0], minPoint[1], maxPoint[2], 1.0f);
        corners[6] = modelMatrix * glm::vec4(maxPoint[0], maxPoint[1], maxPoint[2], 1.0f);
        corners[7] = modelMatrix * glm::vec4(minPoint[0], maxPoint[1], maxPoint[2], 1.0f);

        glm::vec4 cornersClip[8];
        for (int i = 0; i < 8; ++i) {
            cornersClip[i] = viewProjectionMatrix * corners[i];
            cornersClip[i] /= cornersClip[i].w;
        }

        bool isVisible = false;
        for (int i = 0; i < 8; ++i) {
            if (cornersClip[i].x >= -1.0f && cornersClip[i].x <= 1.0f && cornersClip[i].y >= -1.0f && cornersClip[i].y <= 1.0f) {
                isVisible = true;
                break;
            }
        }

        glm::vec2 minPointScreen = glm::vec2(std::numeric_limits<float>::max());
        glm::vec2 maxPointScreen = glm::vec2(std::numeric_limits<float>::min());

        for (int i = 0; i < 8; ++i) {
            minPointScreen.x = std::min(minPointScreen.x, (cornersClip[i].x + 1.0f) * 0.5f * screenWidth);
            minPointScreen.y = std::min(minPointScreen.y, (1.0f - cornersClip[i].y) * 0.5f * screenHeight);
            maxPointScreen.x = std::max(maxPointScreen.x, (cornersClip[i].x + 1.0f) * 0.5f * screenWidth);
            maxPointScreen.y = std::max(maxPointScreen.y, (1.0f - cornersClip[i].y) * 0.5f * screenHeight);
        }

        const float margin = 5.0f;
        minPointScreen -= glm::vec2(margin, margin);
        maxPointScreen += glm::vec2(margin, margin);

        gui->DrawRoundedRect(minPointScreen.x, minPointScreen.y,
                             maxPointScreen.x - minPointScreen.x,
                             maxPointScreen.y - minPointScreen.y,
                             glm::vec4(0.0f, 0.0f, 0.0f, 0.1f));

        if(isVisible) { 
            float xmin = minPointScreen.x;
            float ymin = minPointScreen.y;
            float xmax = maxPointScreen.x;
            float ymax = maxPointScreen.y;
            annotationFile << rob->getName()  << " " << xmin << " " << ymin << " " << xmax << " " << ymax << "\n";
        }
        
        if (isVisible) {
            cv::Mat instanceMask = cv::Mat::zeros(screenHeight, screenWidth, CV_8UC3);
            cv::Vec3b color = getRandomColor();  // Get a unique color for this instance
            
            for (size_t i = 0; i < x->size(); i += 3) {
                const auto& v1 = (*x)[i];
                const auto& v2 = (*x)[i + 1];
                const auto& v3 = (*x)[i + 2];

                glm::vec4 clip1 = viewProjectionMatrix * modelMatrix * glm::vec4(v1[0], v1[1], v1[2], 1.0f);
                glm::vec4 clip2 = viewProjectionMatrix * modelMatrix * glm::vec4(v2[0], v2[1], v2[2], 1.0f);
                glm::vec4 clip3 = viewProjectionMatrix * modelMatrix * glm::vec4(v3[0], v3[1], v3[2], 1.0f);

                clip1 /= clip1.w;
                clip2 /= clip2.w;
                clip3 /= clip3.w;

                if (clip1.x >= -1.0f && clip1.x <= 1.0f && clip1.y >= -1.0f && clip1.y <= 1.0f &&
                    clip2.x >= -1.0f && clip2.x <= 1.0f && clip2.y >= -1.0f && clip2.y <= 1.0f &&
                    clip3.x >= -1.0f && clip3.x <= 1.0f && clip3.y >= -1.0f && clip3.y <= 1.0f) {

                    glm::vec2 screen1 = glm::vec2((clip1.x + 1.0f) * 0.5f * screenWidth, (1.0f - clip1.y) * 0.5f * screenHeight);
                    glm::vec2 screen2 = glm::vec2((clip2.x + 1.0f) * 0.5f * screenWidth, (1.0f - clip2.y) * 0.5f * screenHeight);
                    glm::vec2 screen3 = glm::vec2((clip3.x + 1.0f) * 0.5f * screenWidth, (1.0f - clip3.y) * 0.5f * screenHeight);

                    cv::line(instanceMask, cv::Point(screen1.x, screen1.y), cv::Point(screen2.x, screen2.y), cv::Scalar(color), 1, cv::LINE_AA);
                    cv::line(instanceMask, cv::Point(screen2.x, screen2.y), cv::Point(screen3.x, screen3.y), cv::Scalar(color), 1, cv::LINE_AA);
                    cv::line(instanceMask, cv::Point(screen3.x, screen3.y), cv::Point(screen1.x, screen1.y), cv::Scalar(color), 1, cv::LINE_AA);
                }
            }
            std::string instanceMaskPath = folderName + "/" + rob->getName() + "_instance.png";
            cv::imwrite(instanceMaskPath, instanceMask);

            semanticSegmentationMask += instanceMask;

            // Update panoptic segmentation mask
            for (int i = 0; i < instanceMask.rows; ++i) {
                for (int j = 0; j < instanceMask.cols; ++j) {
                    if (instanceMask.at<cv::Vec3b>(i, j) != cv::Vec3b(0, 0, 0)) {
                        panopticSegmentationMask.at<cv::Vec3b>(i, j) = color;
                    }
                }
            }

            std::string pointCloudPath = folderName + "/" + rob->getName() + ".pcd";
            std::ofstream pointCloudFile(pointCloudPath);
            if (pointCloudFile.is_open()) {
                pointCloudFile << "# .PCD v0.7 - Point Cloud Data file format\n";
                pointCloudFile << "VERSION 0.7\n";
                pointCloudFile << "FIELDS x y z\n";
                pointCloudFile << "SIZE 4 4 4\n";
                pointCloudFile << "TYPE F F F\n";
                pointCloudFile << "COUNT 1 1 1\n";
                pointCloudFile << "WIDTH " << x->size() << "\n";
                pointCloudFile << "HEIGHT 1\n";
                pointCloudFile << "VIEWPOINT 0 0 0 1 0 0 0\n";
                pointCloudFile << "POINTS " << x->size() << "\n";
                pointCloudFile << "DATA ascii\n";
                for (const auto& vertex : *x) {
                    pointCloudFile << vertex[0] << " " << vertex[1] << " " << vertex[2] << "\n";
                }
                pointCloudFile.close();
            }
        }
    }

    annotationFile.close();
    std::string semanticMaskPath = folderName + "/semantic_segmentation.png";
    cv::imwrite(semanticMaskPath, semanticSegmentationMask);
    std::string panopticMaskPath = folderName + "/panoptic_segmentation.png";
    cv::imwrite(panopticMaskPath, panopticSegmentationMask);
}

      
    
    
    
    
    // Battery Panel 
    float AISoffset = offset;
    gui->DoPanel(10.f, offset, 160.f, 126.f);
    offset += 5.f;
    gui->DoLabel(15.f, offset, "ROBOT STATUS");
    offset += 15.f;
    
    id.owner = 5;
    id.item = 0;
    std::vector<std::string> options_batteries;
    unsigned int selected_bat = 0;
    unsigned int newSelected_bat = 0;
    //Add robots to the list
    unsigned int rid_bat = 0;
    while((rob = getSimulationManager()->getRobot(rid_bat)) != nullptr)
    {
        options_batteries.push_back(rob->getName());
        selected_bat = (unsigned int)(rid_bat + 1);
        ++rid_bat;
    }

    newSelected_bat = gui->DoComboBox(id, 15.f, offset, 150.f, options_batteries, selected_bat, "Batteries");
    
    offset += 51.f;
    // batteryLevel is a variable representing the current battery level (between 0 and 100)
    float batteryLevel = getSimulationManager()->getRobot(newSelected_bat)->getBatteryLevel();
    // Calculate the width of the battery bar based on the battery level
    float barWidth = (batteryLevel / 100.f) * 130.f; // Adjust 130.f to suit your panel width
    // Draw the battery outline
    gui->DrawRoundedRect(20.f, offset, 130.f, 20.f, glm::vec4(0.0f, 0.0f, 0.0f, 1.0f)); // Black color
    // Draw the filled battery level bar
    gui->DrawRoundedRect(20.f, offset, barWidth, 20.f, glm::vec4(0.0f, 1.0f, 0.0f, 1.0f)); // Green color
    // Write the percentage inside the battery bar
    char batteryText[16];
    sprintf(batteryText, "%.0f%%", batteryLevel); // Convert float to string
    int numCharacters = strlen(batteryText);
    float textWidth = numCharacters * 8.0f; // Adjust the constant value according to font size and style
    gui->DoLabel(20.f + barWidth / 2 - textWidth / 2, offset + 3, batteryText); // Adjust position for centering
    
    //Bottom panel
    gui->DoPanel(-10, getWindowHeight()-30.f, getWindowWidth()+20, 30.f);
    
    std::sprintf(buf, "Drawing time: %1.2lf (%1.2lf) ms", getDrawingTime(), getDrawingTime(true));
    gui->DoLabel(10, getWindowHeight() - 20.f, buf);
    
    std::sprintf(buf, "CPU usage: %1.0lf%%", getSimulationManager()->getCpuUsage());
    gui->DoLabel(190, getWindowHeight() - 20.f, buf);
    
    std::sprintf(buf, "Simulation time: %1.2lf s", getSimulationManager()->getSimulationTime());
    gui->DoLabel(320, getWindowHeight() - 20.f, buf);

    gui->DoLabel(getWindowWidth() - 100.f, getWindowHeight() - 20.f, "Hit [K] for keymap");

    //Keymap
    if(displayKeymap)
    {
        offset = getWindowHeight()-278.f;
        GLfloat left = getWindowWidth()-130.f; 
        gui->DoPanel(left - 10.f, offset, 130.f, 238.f); offset += 10.f;
        gui->DoLabel(left, offset, "[H] show/hide GUI"); offset += 16.f;
        gui->DoLabel(left, offset, "[C] show/hide console"); offset += 16.f;
        gui->DoLabel(left, offset, "[W] move forward"); offset += 16.f;
        gui->DoLabel(left, offset, "[S] move backward"); offset += 16.f;
        gui->DoLabel(left, offset, "[A] move left"); offset += 16.f;
        gui->DoLabel(left, offset, "[D] move right"); offset += 16.f;
        //gui->DoLabel(left, offset, "[Q] move up"); offset += 16.f;
        //gui->DoLabel(left, offset, "[Z] move down"); offset += 16.f;
        gui->DoLabel(left, offset, "[Shift] move fast"); offset += 16.f;
        gui->DoLabel(left, offset, "[Mouse right] rotate"); offset += 16.f;
        gui->DoLabel(left, offset, "[Mouse middle] move"); offset += 16.f;
        gui->DoLabel(left, offset, "[Mouse scroll] zoom"); offset += 16.f;
        gui->DoLabel(left, offset, "[R] move up"); offset += 16.f;
        gui->DoLabel(left, offset, "[F] move down"); offset += 16.f;
        gui->DoLabel(left, offset, "[Q] Rotate left"); offset += 16.f;
        gui->DoLabel(left, offset, "[E] Rotate right"); offset += 16.f;
    }

    //Performance
    if(displayPerformance)
    {
        std::vector<std::vector<GLfloat> > perfData;    
        perfData.push_back(getSimulationManager()->getPerformanceMonitor().getPhysicsTimeHistory<GLfloat>(100));
        perfData.push_back(getSimulationManager()->getPerformanceMonitor().getHydrodynamicsTimeHistory<GLfloat>(100));

        id.owner = 6;
        id.item = 0;
        gui->DoTimePlot(id, getWindowWidth()-300, getWindowHeight()-200, 290, 160, perfData, "Performance Monitor", new Scalar[2]{-1, 10000});
    }
}

void GraphicalSimulationApp::StartSimulation()
{
    SimulationApp::StartSimulation();
    
    GraphicalSimulationThreadData* data = new GraphicalSimulationThreadData();
    data->app = this;
    data->drawingQueueMutex = glPipeline->getDrawingQueueMutex();
    simulationThread = SDL_CreateThread(GraphicalSimulationApp::RunSimulation, "simulationThread", data);
}

void GraphicalSimulationApp::ResumeSimulation()
{
    SimulationApp::ResumeSimulation();
    
    GraphicalSimulationThreadData* data = new GraphicalSimulationThreadData();
    data->app = this;
    data->drawingQueueMutex = glPipeline->getDrawingQueueMutex();
    simulationThread = SDL_CreateThread(GraphicalSimulationApp::RunSimulation, "simulationThread", data);
}

void GraphicalSimulationApp::StopSimulation()
{
    SimulationApp::StopSimulation();
	selectedEntity = std::make_pair(nullptr, -1);
	trackballCenter = NULL;
    
    int status;
    SDL_WaitThread(simulationThread, &status);
    simulationThread = NULL;
}

void GraphicalSimulationApp::CleanUp()
{
    SimulationApp::CleanUp();
    glDeleteQueries(2, timeQuery);

    if(joystick != NULL)
        SDL_JoystickClose(0);
    
    if(glLoadingContext != NULL)
        SDL_GL_DeleteContext(glLoadingContext);
    
    SDL_GL_DeleteContext(glMainContext);
    SDL_DestroyWindow(window);
    SDL_Quit();
}

int GraphicalSimulationApp::RenderLoadingScreen(void* data)
{
    //Get application
    LoadingThreadData* ltdata = (LoadingThreadData*)data;
    
    //Make drawing in this thread possible
    SDL_GL_MakeCurrent(ltdata->app->window, ltdata->app->glLoadingContext);  
    
    //Render loading screen
    glClearColor(0.2f, 0.2f, 0.2f, 0.0f);
    glScissor(0, 0, ltdata->app->windowW, ltdata->app->windowH);
    glViewport(0, 0, ltdata->app->windowW, ltdata->app->windowH);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    
    GLuint vao;
    glGenVertexArrays(1, &vao);
    glBindVertexArray(vao);
    glEnableVertexAttribArray(0);
    
    while(ltdata->app->loading)
    {
        glClear(GL_COLOR_BUFFER_BIT);
        
        //Lock to prevent adding lines to the console while rendering
        SDL_LockMutex(ltdata->mutex);
        ((OpenGLConsole*)ltdata->app->getConsole())->Render(false);
        SDL_UnlockMutex(ltdata->mutex);
        
        SDL_GL_SwapWindow(ltdata->app->window);
    }
    
    glBindVertexArray(0);
    glDeleteVertexArrays(1, &vao);
    
    //Detach thread from GL context
    SDL_GL_MakeCurrent(ltdata->app->window, NULL);
    return 0;
}

int GraphicalSimulationApp::RunSimulation(void* data)
{
    GraphicalSimulationThreadData* stdata = (GraphicalSimulationThreadData*)data;
    SimulationManager* sim = stdata->app->getSimulationManager();

    int maxThreads = std::max(omp_get_max_threads()/2, 1);
    omp_set_num_threads(maxThreads);
    
    while(stdata->app->isRunning())
    {
        sim->AdvanceSimulation();
        if(stdata->app->getGLPipeline()->isDrawingQueueEmpty())
        {
            SDL_LockMutex(stdata->drawingQueueMutex);
            sim->UpdateDrawingQueue();
            SDL_UnlockMutex(stdata->drawingQueueMutex);
        }
    }
    
    return 0;
}

void GraphicalSimulationApp::StopSimulationWrapper() {
        StopSimulation();
}

void GraphicalSimulationApp::ResumeSimulationWrapper() {
        ResumeSimulation();
}

int  GraphicalSimulationApp::getTurbidityLevel(){
     return turbidity_level;
}

}
