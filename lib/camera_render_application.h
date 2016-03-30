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
    CameraRenderApplication(std::string resourcePath, double cam_fx = 400, double cam_fy = 400);
    virtual ~CameraRenderApplication(void);

    void getCameraIntrinsics(double* fx, double* fy, double* cx, double* cy);
    void getCameraPosition(double* x, double* y, double* z);
    void getCameraOrientation(double* w, double* x, double* y, double* z);
    void setCameraPosition(double x, double y, double z);
    void setCameraOrientation(double w, double x, double y, double z);
    void setCameraLookAt(double x, double y, double z);
    void loadModel(std::string entity_name, std::string filename);
    void createCylinder(double x, double y, double z, double h, double r);
    void getMeshInformation(
         Ogre::Entity* entity,
         size_t &vertex_count,
         Ogre::Vector3 *&vertices,
         size_t &index_count, 
         unsigned long *&indices,
         const Ogre::Vector3 &position,
         const Ogre::Quaternion &orient,
         const Ogre::Vector3 &scale);
    Ogre::Light* main_light;
    Ogre::Entity* model;
protected:
    virtual void createScene(void);
    int cyl_id;
    bool model_loaded;
};

//---------------------------------------------------------------------------

#endif // #ifndef __CameraRenderApplication_h_

//---------------------------------------------------------------------------
