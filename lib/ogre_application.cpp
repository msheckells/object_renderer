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
    vp->setBackgroundColour(Ogre::ColourValue(0.0,0.0,0.0));

    // Alter the camera aspect ratio to match the viewport
    mCamera->setAspectRatio(Ogre::Real(vp->getActualWidth()) / Ogre::Real(vp->getActualHeight()));
}

void OgreApplication::createDepthRTT(void)
{
  Ogre::TexturePtr texPtr = 
    Ogre::TextureManager::getSingleton().createManual(
      "DepthMap", 
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, 
      Ogre::TEX_TYPE_2D, 
      mWindow->getWidth(), mWindow->getHeight(), 
      0, 
      Ogre::PF_FLOAT32_R, 
      Ogre::TU_RENDERTARGET);

  Ogre::RenderTexture *depth_map = texPtr->getBuffer()->getRenderTarget();
  depth_map->addViewport(mCamera);
  depth_map->getViewport(0)->setClearEveryFrame(true);
  depth_map->getViewport(0)->setBackgroundColour(Ogre::ColourValue::Black);
  depth_map->getViewport(0)->setOverlaysEnabled(false);

  Ogre::MaterialPtr matPtr = Ogre::MaterialManager::getSingleton().create("DepthMapMat", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  matPtr->getTechnique(0)->getPass(0)->createTextureUnitState("DepthMap");
  matPtr->getTechnique(0)->getPass(0)->setDepthCheckEnabled(false);
  matPtr->getTechnique(0)->getPass(0)->setDepthWriteEnabled(false);
  matPtr->getTechnique(0)->getPass(0)->setLightingEnabled(false);

  Ogre::HardwarePixelBufferSharedPtr pixelBuffer =  texPtr->getBuffer();
  mDepthBuffer = new unsigned char[mWindow->getWidth()*mWindow->getHeight()*pixelBuffer->getFormat()];

  this->mDepthMaterial = (Ogre::Material*)Ogre::MaterialManager::getSingleton().getByName("Ogre/DepthMap").get();
  this->mDepthMaterial->load();
  depth_map->addListener(this);  
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

void OgreApplication::preRenderTargetUpdate(const Ogre::RenderTargetEvent& rte)
{
  Ogre::TexturePtr texPtr = Ogre::TextureManager::getSingleton().getByName("DepthMap");
  Ogre::RenderTexture* depth_map = texPtr->getBuffer()->getRenderTarget();

  mSceneMgr->_suppressRenderStateChanges(true);
  mSceneMgr->_setPass(mDepthMaterial->getBestTechnique()->getPass(0), true, false);
  mRenderSys->_setViewport(depth_map->getViewport(0));
  mRenderSys->_setProjectionMatrix(mCamera->getProjectionMatrixRS());
  mRenderSys->_setViewMatrix(mCamera->getViewMatrix(true));
}
 
void OgreApplication::postRenderTargetUpdate(const Ogre::RenderTargetEvent& rte)
{
  Ogre::TexturePtr texPtr = Ogre::TextureManager::getSingleton().getByName("DepthMap");
  Ogre::HardwarePixelBufferSharedPtr pixelBuffer =  texPtr->getBuffer();

  mSceneMgr->_suppressRenderStateChanges(false);
  Ogre::Box extents(0, 0, mWindow->getWidth(), mWindow->getHeight());
  Ogre::PixelBox pb(extents, pixelBuffer->getFormat(), mDepthBuffer);
  pixelBuffer->blitToMemory(pb);
}


void OgreApplication::saveDepthMap(std::string filename)
{
  Ogre::TexturePtr texPtr = Ogre::TextureManager::getSingleton().getByName("DepthMap");
  Ogre::RenderTexture* depth_map = texPtr->getBuffer()->getRenderTarget();
  depth_map->writeContentsToFile(filename);
}

bool OgreApplication::renderOnce(void)
{
  Ogre::WindowEventUtilities::messagePump();

  // Update depth RTT
  Ogre::TexturePtr texPtr = Ogre::TextureManager::getSingleton().getByName("DepthMap");
  Ogre::RenderTexture* depth_map = texPtr->getBuffer()->getRenderTarget();
  depth_map->update();

  return mRoot->renderOneFrame();
}

size_t OgreApplication::getBytesPerPixel(void)
{
  return Ogre::PixelUtil::getNumElemBytes(Ogre::PF_BYTE_RGBA/*Ogre::PF_L8*/);
}

size_t OgreApplication::getBytesPerDepthPixel(void)
{
  return Ogre::PixelUtil::getNumElemBytes(Ogre::PF_FLOAT32_R/*Ogre::PF_L8*/);
}

void OgreApplication::getDepthData(unsigned char* data)
{
  Ogre::TexturePtr texPtr = Ogre::TextureManager::getSingleton().getByName("DepthMap");
  Ogre::HardwarePixelBufferSharedPtr pixelBuffer =  texPtr->getBuffer();
  size_t len = mWindow->getWidth()*mWindow->getHeight()*this->getBytesPerDepthPixel();
  
  if(data)
  {
    memcpy(data, mDepthBuffer, len);
  }
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
    // Get pointer to rendering system

    setupResources();

    bool carryOn = configure();
    if (!carryOn) return false;

    chooseSceneManager();
    createCamera();
    createViewports();
    this->mRenderSys = mRoot->getRenderSystem();

    // Set default mipmap level (NB some APIs ignore this)
    Ogre::TextureManager::getSingleton().setDefaultNumMipmaps(5);

    // Load resources
    loadResources();

    createDepthRTT();

    // Create the scene
    createScene();

    return true;
};
//---------------------------------------------------------------------------
