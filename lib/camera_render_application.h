/*
-----------------------------------------------------------------------------
Filename:    CameraRenderApplication.h
-----------------------------------------------------------------------------

This source file is part of the
   ___                 __    __ _ _    _
  /___\__ _ _ __ ___  / / /\ \ (_) | _(_)
 //  // _` | '__/ _ \ \ \/  \/ / | |/ / |
/ \_// (_| | | |  __/  \  /\  /| |   <| |
\___/ \__, |_|  \___|   \/  \/ |_|_|\_\_|
      |___/
Tutorial Framework (for Ogre 1.9)
http://www.ogre3d.org/wiki/
-----------------------------------------------------------------------------
*/

#ifndef __CameraRenderApplication_h_
#define __CameraRenderApplication_h_

#include "ogre_application.h"

//---------------------------------------------------------------------------

class CameraRenderApplication : public OgreApplication
{
public:
    CameraRenderApplication(std::string resourcePath);
    virtual ~CameraRenderApplication(void);

    void getCameraIntrinsics(double* fx, double* fy, double* cx, double* cy);
    void getCameraPosition(double* x, double* y, double* z);
    void getCameraOrientation(double* w, double* x, double* y, double* z);
    void setCameraPosition(double x, double y, double z);
    void setCameraOrientation(double w, double x, double y, double z);
    void setCameraLookAt(double x, double y, double z);
    void loadModel(std::string entity_name, std::string filename);
    Ogre::Light* main_light;
protected:
    virtual void createScene(void);
};

//---------------------------------------------------------------------------

#endif // #ifndef __CameraRenderApplication_h_

//---------------------------------------------------------------------------
