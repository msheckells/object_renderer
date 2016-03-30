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
#include <OgreSubMesh.h>
#include <OgreSubEntity.h>

//---------------------------------------------------------------------------
CameraRenderApplication::CameraRenderApplication(std::string resourcePath, double cam_fx, double cam_fy) :
  OgreApplication(resourcePath),
  cyl_id(0),
  model_loaded(false),
  model(NULL)
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
  assert(!model_loaded);
  model_loaded = true;
 // Create an Entity
  model = mSceneMgr->createEntity(entity_name, filename);
  //obj->setMaterialName("Ogre/DepthMap");
  // Create a SceneNode and attach the Entity to it
  Ogre::SceneNode* objNode = mSceneMgr->getRootSceneNode()->createChildSceneNode(entity_name+"Node");
  objNode->attachObject(model);
}

void CameraRenderApplication::getMeshInformation(
         Ogre::Entity* entity,
         size_t &vertex_count,
         Ogre::Vector3 *&vertices,
         size_t &index_count, 
         unsigned long *&indices,
         const Ogre::Vector3 &position,
         const Ogre::Quaternion &orient,
         const Ogre::Vector3 &scale
)
{
   bool added_shared = false;
   size_t current_offset = 0;
   size_t shared_offset = 0;
   size_t next_offset = 0;
   size_t index_offset = 0;
   vertex_count = index_count = 0;

   Ogre::MeshPtr mesh = entity->getMesh();


   bool useSoftwareBlendingVertices = entity->hasSkeleton();

   if (useSoftwareBlendingVertices)
   {
      entity->_updateAnimation();
   }

   // Calculate how many vertices and indices we're going to need
   for (unsigned short i = 0; i < mesh->getNumSubMeshes(); ++i)
   {
      Ogre::SubMesh* submesh = mesh->getSubMesh( i );

      // We only need to add the shared vertices once
      if(submesh->useSharedVertices)
      {
         if( !added_shared )
         {
            vertex_count += mesh->sharedVertexData->vertexCount;
            added_shared = true;
         }
      }
      else
      {
         vertex_count += submesh->vertexData->vertexCount;
      }

      // Add the indices
      index_count += submesh->indexData->indexCount;
   }


   // Allocate space for the vertices and indices
   vertices = new Ogre::Vector3[vertex_count];
   indices = new unsigned long[index_count];

   added_shared = false;

   // Run through the submeshes again, adding the data into the arrays
   for ( unsigned short i = 0; i < mesh->getNumSubMeshes(); ++i)
   {
      Ogre::SubMesh* submesh = mesh->getSubMesh(i);

      //----------------------------------------------------------------
      // GET VERTEXDATA
      //----------------------------------------------------------------

      //Ogre::VertexData* vertex_data = submesh->useSharedVertices ? mesh->sharedVertexData : submesh->vertexData;
      Ogre::VertexData* vertex_data;

      //When there is animation:
      if(useSoftwareBlendingVertices)
#ifdef BUILD_AGAINST_AZATHOTH
         vertex_data = submesh->useSharedVertices ? entity->_getSharedBlendedVertexData() : entity->getSubEntity(i)->_getBlendedVertexData();
#else
         vertex_data = submesh->useSharedVertices ? entity->_getSkelAnimVertexData() : entity->getSubEntity(i)->_getSkelAnimVertexData();
#endif
      else
         vertex_data = submesh->useSharedVertices ? mesh->sharedVertexData : submesh->vertexData;


      if((!submesh->useSharedVertices)||(submesh->useSharedVertices && !added_shared))
      {
         if(submesh->useSharedVertices)
         {
            added_shared = true;
            shared_offset = current_offset;
         }

         const Ogre::VertexElement* posElem =
            vertex_data->vertexDeclaration->findElementBySemantic(Ogre::VES_POSITION);

         Ogre::HardwareVertexBufferSharedPtr vbuf =
            vertex_data->vertexBufferBinding->getBuffer(posElem->getSource());

         unsigned char* vertex =
            static_cast<unsigned char*>(vbuf->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));

         // There is _no_ baseVertexPointerToElement() which takes an Ogre::Real or a double
         //  as second argument. So make it float, to avoid trouble when Ogre::Real will
         //  be comiled/typedefed as double:
         //      Ogre::Real* pReal;
         float* pReal;

         for( size_t j = 0; j < vertex_data->vertexCount; ++j, vertex += vbuf->getVertexSize())
         {
            posElem->baseVertexPointerToElement(vertex, &pReal);

            Ogre::Vector3 pt(pReal[0], pReal[1], pReal[2]);

            vertices[current_offset + j] = (orient * (pt * scale)) + position;
         }

         vbuf->unlock();
         next_offset += vertex_data->vertexCount;
      }


      Ogre::IndexData* index_data = submesh->indexData;
      size_t numTris = index_data->indexCount / 3;
      Ogre::HardwareIndexBufferSharedPtr ibuf = index_data->indexBuffer;

      bool use32bitindexes = (ibuf->getType() == Ogre::HardwareIndexBuffer::IT_32BIT);

      unsigned long*  pLong = static_cast<unsigned long*>(ibuf->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));
      unsigned short* pShort = reinterpret_cast<unsigned short*>(pLong);


      size_t offset = (submesh->useSharedVertices)? shared_offset : current_offset;
      size_t index_start = index_data->indexStart;
      size_t last_index = numTris*3 + index_start;

      if (use32bitindexes)
         for (size_t k = index_start; k < last_index; ++k)
         {
            indices[index_offset++] = pLong[k] + static_cast<unsigned long>( offset );
         }

      else
         for (size_t k = index_start; k < last_index; ++k)
         {
            indices[ index_offset++ ] = static_cast<unsigned long>( pShort[k] ) +
               static_cast<unsigned long>( offset );
         }

         ibuf->unlock();
         current_offset = next_offset;
   }
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
