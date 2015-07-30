/*
-----------------------------------------------------------------------------
Filename:    OgreApplication.h
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

#ifndef __OgreApplication_h_
#define __OgreApplication_h_

#include <OgreCamera.h>
#include <OgreEntity.h>
#include <OgreLogManager.h>
#include <OgreRoot.h>
#include <OgreViewport.h>
#include <OgreSceneManager.h>
#include <OgreRenderWindow.h>
#include <OgreConfigFile.h>
#include <OgreRenderTargetListener.h>

#  include <OISEvents.h>
#  include <OISInputManager.h>
#  include <OISKeyboard.h>
#  include <OISMouse.h>

//#  include <SdkTrays.h>
//#  include <SdkCameraMan.h>

#ifdef OGRE_STATIC_LIB
#  define OGRE_STATIC_GL
#  if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
#    define OGRE_STATIC_Direct3D9
// D3D10 will only work on vista, so be careful about statically linking
#    if OGRE_USE_D3D10
#      define OGRE_STATIC_Direct3D10
#    endif
#  endif
#  define OGRE_STATIC_BSPSceneManager
#  define OGRE_STATIC_ParticleFX
#  define OGRE_STATIC_CgProgramManager
#  ifdef OGRE_USE_PCZ
#    define OGRE_STATIC_PCZSceneManager
#    define OGRE_STATIC_OctreeZone
#  else
#    define OGRE_STATIC_OctreeSceneManager
#  endif
#  include "OgreStaticPluginLoader.h"
#endif

//---------------------------------------------------------------------------

class OgreApplication : public Ogre::RenderTargetListener
{
public:
    OgreApplication(std::string resourcePath);
    virtual ~OgreApplication(void);

    virtual void go(void);
    virtual bool renderOnce(void);
    virtual void destroyScene(void);
    virtual void getRenderData(int width, int height, unsigned char* data);
    virtual void getDepthData(unsigned char* data);
    virtual void saveRenderToFile(std::string filename);
    size_t getBytesPerPixel(void);
    size_t getBytesPerDepthPixel(void);
    int getWindowWidth(void);
    int getWindowHeight(void);
    void saveDepthMap(std::string filename);

    unsigned char* mDepthBuffer;
protected:
    virtual void preRenderTargetUpdate(const Ogre::RenderTargetEvent& rte);
    virtual void postRenderTargetUpdate(const Ogre::RenderTargetEvent& rte);

    virtual bool setup();
    virtual bool configure(void);
    virtual void chooseSceneManager(void);
    virtual void createCamera(void);
    virtual void createDepthRTT(void);
    virtual void createScene(void) = 0; // Override me!
    virtual void createViewports(void);
    virtual void setupResources(void);
    virtual void loadResources(void);

    Ogre::Root*                 mRoot;
    Ogre::Camera*               mCamera;
    Ogre::SceneManager*         mSceneMgr;
    Ogre::RenderWindow*         mWindow;
    Ogre::String                mResourcesCfg;
    Ogre::String                mPluginsCfg;
    Ogre::Material*             mDepthMaterial;
    Ogre::RenderSystem*             mRenderSys;

    Ogre::String                 m_ResourcePath;
 
    double fx;
    double fy;

#ifdef OGRE_STATIC_LIB
    Ogre::StaticPluginLoader m_StaticPluginLoader;
#endif
};

//---------------------------------------------------------------------------

#endif // #ifndef __OgreApplication_h_

//---------------------------------------------------------------------------
