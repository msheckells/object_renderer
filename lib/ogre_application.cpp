/*
-----------------------------------------------------------------------------
Filename:    OgreApplication.cpp
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

#include "ogre_application.h"
#include <OgreLogManager.h>

#if OGRE_PLATFORM == OGRE_PLATFORM_APPLE
#include <macUtils.h>
#endif

//---------------------------------------------------------------------------
OgreApplication::OgreApplication(void)
    : mRoot(0),
    mCamera(0),
    mSceneMgr(0),
    mWindow(0),
    mResourcesCfg(Ogre::StringUtil::BLANK),
    mPluginsCfg(Ogre::StringUtil::BLANK)
{
    m_ResourcePath = "../../cfg/";
}

//---------------------------------------------------------------------------
OgreApplication::~OgreApplication(void)
{
    delete mRoot;
}

//---------------------------------------------------------------------------
bool OgreApplication::configure(void)
{
    // Show the configuration dialog and initialise the system.
    // You can skip this and use root.restoreConfig() to load configuration
    // settings if you were sure there are valid ones saved in ogre.cfg.
    if(mRoot->showConfigDialog())
    {
        //mRoot->restoreConfig();
        // If returned true, user clicked OK so initialise.
        // Here we choose to let the system create a default rendering window by passing 'true'.
        mWindow = mRoot->initialise(true, "OGRE Render Window");

        return true;
    }
  /*  else
    {
        return false;
    }*/
}
//---------------------------------------------------------------------------
void OgreApplication::chooseSceneManager(void)
{
    // Get the SceneManager, in this case a generic one
    mSceneMgr = mRoot->createSceneManager(Ogre::ST_GENERIC);
}
//---------------------------------------------------------------------------
void OgreApplication::createCamera(void)
{
    // Create the camera
    mCamera = mSceneMgr->createCamera("PlayerCam");

    // Position it at 500 in Z direction
    mCamera->setPosition(Ogre::Vector3(0,0,80));
    // Look back along -Z
    mCamera->lookAt(Ogre::Vector3(0,0,-300));
    mCamera->setNearClipDistance(5);

}
//---------------------------------------------------------------------------

void OgreApplication::destroyScene(void)
{
}

int OgreApplication::getWindowWidth(void)
{
  return mWindow->getWidth();
}

int OgreApplication::getWindowHeight(void)
{
  return mWindow->getHeight();
}

//---------------------------------------------------------------------------
void OgreApplication::createViewports(void)
{
    // Create one viewport, entire window
    Ogre::Viewport* vp = mWindow->addViewport(mCamera);
    vp->setBackgroundColour(Ogre::ColourValue(0.4,0.4,0.4));

    // Alter the camera aspect ratio to match the viewport
    mCamera->setAspectRatio(Ogre::Real(vp->getActualWidth()) / Ogre::Real(vp->getActualHeight()));
}
//---------------------------------------------------------------------------
void OgreApplication::setupResources(void)
{
    // Load resource paths from config file
    Ogre::ConfigFile cf;
    cf.load(mResourcesCfg);

    // Go through all sections & settings in the file
    Ogre::ConfigFile::SectionIterator seci = cf.getSectionIterator();

    Ogre::String secName, typeName, archName;
    while (seci.hasMoreElements())
    {
        secName = seci.peekNextKey();
        Ogre::ConfigFile::SettingsMultiMap *settings = seci.getNext();
        Ogre::ConfigFile::SettingsMultiMap::iterator i;
        for (i = settings->begin(); i != settings->end(); ++i)
        {
            typeName = i->first;
            archName = i->second;

            Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
                archName, typeName, secName);
        }
    }
}
//---------------------------------------------------------------------------
void OgreApplication::loadResources(void)
{
    Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();
}
//---------------------------------------------------------------------------
void OgreApplication::go(void)
{
    mResourcesCfg = m_ResourcePath + "resources.cfg";
    mPluginsCfg = m_ResourcePath + "plugins.cfg";

    if (!setup())
        return;

    //mRoot->startRendering();

    // Clean up
    //destroyScene();
}

bool OgreApplication::renderOnce(void)
{
  Ogre::WindowEventUtilities::messagePump();
  return mRoot->renderOneFrame();
}

size_t OgreApplication::getBytesPerPixel(void)
{
  return Ogre::PixelUtil::getNumElemBytes(Ogre::PF_BYTE_RGBA/*Ogre::PF_L8*/);
}

void OgreApplication::getRenderData(int width, int height, unsigned char* data)
{
  Ogre::Box extents(0, 0, width, height);
  Ogre::PixelBox pb(extents, Ogre::PF_BYTE_RGBA/*Ogre::PF_L8*/, data);
  mWindow->copyContentsToMemory(pb);
}

void OgreApplication::saveRenderToFile(std::string filename)
{
  mWindow->writeContentsToFile(filename);
}

//---------------------------------------------------------------------------
bool OgreApplication::setup(void)
{
    Ogre::LogManager * lm = new Ogre::LogManager();
    lm->createLog("./ogre.log", true, false, false); 
     
    mRoot = new Ogre::Root(mPluginsCfg);

    setupResources();

    bool carryOn = configure();
    if (!carryOn) return false;

    chooseSceneManager();
    createCamera();
    createViewports();

    // Set default mipmap level (NB some APIs ignore this)
    Ogre::TextureManager::getSingleton().setDefaultNumMipmaps(5);

    // Load resources
    loadResources();

    // Create the scene
    createScene();

    return true;
};
//---------------------------------------------------------------------------
