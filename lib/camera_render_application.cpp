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

//---------------------------------------------------------------------------
CameraRenderApplication::CameraRenderApplication(std::string resourcePath) :
  OgreApplication(resourcePath)
{
}
//---------------------------------------------------------------------------
CameraRenderApplication::~CameraRenderApplication(void)
{
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
  Ogre::Quaternion r(Ogre::Degree(90), Ogre::Vector3::UNIT_X);
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

//---------------------------------------------------------------------------
/*
#ifdef __cplusplus
extern "C" {
#endif

    int main(int argc, char *argv[])
    {
        // Create application object
        CameraRenderApplication app;
        int width = 752;
        int height = 480;
        size_t bytesPerPixel = app.getBytesPerPixel();
        try {
            app.go();
            unsigned char* data = new unsigned char[width*height*bytesPerPixel];
            while(app.renderOnce())
            {
              app.getRenderData(width, height, data);     
            }
            app.destroyScene();
        } catch(Ogre::Exception& e)  {
            std::cerr << "An exception has occurred: " <<
                e.getFullDescription().c_str() << std::endl;
        }

        return 0;
    }

#ifdef __cplusplus
}
#endif
*/
//---------------------------------------------------------------------------
