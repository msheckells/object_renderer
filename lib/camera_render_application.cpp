/*
-----------------------------------------------------------------------------
Filename:    CameraRenderApplication.cpp
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
#include "camera_render_application.h"
#include <OgreManualObject.h>

//---------------------------------------------------------------------------
CameraRenderApplication::CameraRenderApplication(std::string resourcePath, double cam_fx, double cam_fy) :
  OgreApplication(resourcePath),
  cyl_id(0)
{
  fx = cam_fx;
  fy = cam_fy;
}
//---------------------------------------------------------------------------
CameraRenderApplication::~CameraRenderApplication(void)
{
}

void CameraRenderApplication::createCylinder(double x, double y, double z, double h, double r)
{
  const int cylinder_circle_resolution = 500;
  Ogre::Degree theta(0);
  Ogre::Degree alpha (360./cylinder_circle_resolution);

  Ogre::Vector3 cylinder_circle1[cylinder_circle_resolution];
  Ogre::Vector3 cylinder_circle2[cylinder_circle_resolution];
  Ogre::Vector3 cylinder_circle1_center;
  Ogre::Vector3 cylinder_circle2_center;

  cylinder_circle1_center.x = x;
  cylinder_circle1_center.y = y;
  cylinder_circle1_center.z = z;
  cylinder_circle2_center = cylinder_circle1_center;
  cylinder_circle2_center.z += h;

  for(int i = 0; i < cylinder_circle_resolution; i++)
  {
    theta += alpha;
    cylinder_circle1[i] = cylinder_circle1_center;
    cylinder_circle1[i].x += r*Ogre::Math::Cos(theta);
    cylinder_circle1[i].y += r*Ogre::Math::Sin(theta);
    cylinder_circle2[i] = cylinder_circle1[i];
    cylinder_circle2[i].z += h;
  }

  Ogre::ManualObject* cylinder = mSceneMgr->createManualObject("Cylinder" + std::to_string(cyl_id++));

  // Face 1
  cylinder->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_TRIANGLE_FAN);
  cylinder->colour(Ogre::ColourValue(0.0f, 0.0f, 1.0f));
  cylinder->position(cylinder_circle1_center);
  for(int i = 0; i < cylinder_circle_resolution; i++)
  {
    cylinder->position(cylinder_circle1[i]);
  }
  cylinder->position(cylinder_circle1[0]);
  cylinder->end();

  // Curved Surface
  cylinder->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_TRIANGLE_FAN);
  cylinder->colour(Ogre::ColourValue(0.2f, 0.6f, 1.0f));
  for(int i = 0; i < cylinder_circle_resolution-1; i++)
  {
    cylinder->position(cylinder_circle1[i]);
    cylinder->position(cylinder_circle2[i]);
    cylinder->position(cylinder_circle1[i+1]);

    cylinder->position(cylinder_circle1[i+1]);
    cylinder->position(cylinder_circle2[i]);
    cylinder->position(cylinder_circle2[i+1]);
  }
  cylinder->position(cylinder_circle1[cylinder_circle_resolution-1]);
  cylinder->position(cylinder_circle2[cylinder_circle_resolution-1]);
  cylinder->position(cylinder_circle1[0]);
  cylinder->position(cylinder_circle1[0]);
  cylinder->position(cylinder_circle2[cylinder_circle_resolution-1]);
  cylinder->position(cylinder_circle2[0]);
  cylinder->end();
  
  // Face 2
  cylinder->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_TRIANGLE_FAN);
  cylinder->colour(Ogre::ColourValue(0.0f, 0.0f, 1.0f));
  cylinder->position(cylinder_circle2_center);
  for(int i = 0; i < cylinder_circle_resolution; i++)
  {
    cylinder->position(cylinder_circle2[i]);
  }
  cylinder->position(cylinder_circle2[0]);
  cylinder->end();

  mSceneMgr->getRootSceneNode()->createChildSceneNode()->attachObject(cylinder);
}

//---------------------------------------------------------------------------
void CameraRenderApplication::createScene(void)
{
  // Create your scene here :)
  // Set the scene's ambient light
  mSceneMgr->setAmbientLight(Ogre::ColourValue(1.0, 1.0, 1.0));
 
  // Create a Light and set its position
  main_light = mSceneMgr->createLight("MainLight");
  main_light->setPosition(0.0f, 0.0f, 0.0f);
  main_light->setType(Ogre::Light::LT_DIRECTIONAL);
  main_light->setAttenuation(10000.0f, 1.0f, 0.0f, 0);
  main_light->setCastShadows(false);
}

void CameraRenderApplication::loadModel(std::string entity_name, std::string filename)
{
 // Create an Entity
  Ogre::Entity* obj = mSceneMgr->createEntity(entity_name, filename);
  //obj->setMaterialName("Ogre/DepthMap");
  // Create a SceneNode and attach the Entity to it
  Ogre::SceneNode* objNode = mSceneMgr->getRootSceneNode()->createChildSceneNode(entity_name+"Node");
  objNode->attachObject(obj);
}

void CameraRenderApplication::getCameraPosition(double* x, double* y , double* z)
{
  Ogre::Vector3 pos =  mCamera->getPosition();
  *x = pos.x;
  *y = pos.y;
  *z = pos.z;
}

void CameraRenderApplication::getCameraOrientation(double* w, double* x, double* y , double* z)
{
  Ogre::Quaternion q =  mCamera->getOrientation();
  Ogre::Quaternion rinv(Ogre::Degree(-180), Ogre::Vector3::UNIT_X);
  q = q*rinv;
  *w = q.w;
  *x = q.x;
  *y = q.y;
  *z = q.z;
}

void CameraRenderApplication::setCameraPosition(double x, double y , double z)
{
  mCamera->setPosition(Ogre::Vector3(x,y,z));
}

void CameraRenderApplication::setCameraOrientation(double w, double x, double y , double z)
{
  Ogre::Quaternion r(Ogre::Degree(180), Ogre::Vector3::UNIT_X);
  mCamera->setOrientation(Ogre::Quaternion(w,x,y,z)*r);
}

void CameraRenderApplication::getCameraIntrinsics(double* fx, double* fy, double* cx, double* cy)
{
  Ogre::Matrix4 projMat = mCamera->getProjectionMatrix();
  if(fx)
  {
    *fx = projMat[0][0]*getWindowWidth()/2.;
  }
  if(fy)
  {
    *fy = projMat[1][1]*getWindowHeight()/2.;
  }
  if(cx)
  {
    *cx = getWindowWidth()/2.;
  }
  if(cy)
  {
    *cy = getWindowHeight()/2.;
  }
}

void CameraRenderApplication::setCameraLookAt(double x, double y , double z)
{
  mCamera->lookAt(Ogre::Vector3(x,y,z));
}
