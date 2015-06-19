#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/gpu/gpu.hpp>
#include <opencv2/nonfree/gpu.hpp>

#include "camera_render_application.h"
#include "virtual_image_handler.h"
#include <Eigen/Core>
#include <Eigen/Dense>
using namespace cv;


int main(int argc, char *argv[])
{

  // Create application object
  CameraRenderApplication app("../../cfg/");
  app.go();
  app.loadModel("model", "box.mesh"); 
  VirtualImageHandler vih(&app);
  

  try 
  {
    int z = -10;
    int y = 0;
    int x = 0;
    double xget, yget, zget;
    double qwget, qxget, qyget, qzget;
    for(x = -2; x <= 2; x++)
    {
      app.main_light->setPosition(x,y,z);
      app.main_light->setDirection(-x,-y,-z);
      Mat im = vih.getVirtualImage(x, y, z, 1, 0, 0, 0);
      std::cout << "Set position= " << x << ", " << y << ", " << z << std::endl;
      app.getCameraPosition(&xget, &yget, &zget);
      std::cout << "Get position= " << xget << ", " << yget << ", " << zget << std::endl;
      getchar();
    }
    x = 0;
    for(y = -2; y <= 2; y++)
    {
      app.main_light->setPosition(x,y,z);
      app.main_light->setDirection(-x,-y,-z);
      Mat im = vih.getVirtualImage(x, y, z, 1, 0, 0, 0);
      std::cout << "Set position= " << x << ", " << y << ", " << z << std::endl;
      app.getCameraPosition(&xget, &yget, &zget);
      std::cout << "Get position= " << xget << ", " << yget << ", " << zget << std::endl;
      getchar();
    }
    y = 0;
    for(z = -12; z <= -8; z++)
    {
      app.main_light->setPosition(x,y,z);
      app.main_light->setDirection(-x,-y,-z);
      Mat im = vih.getVirtualImage(x, y, z, 1, 0, 0, 0);
      std::cout << "Set position= " << x << ", " << y << ", " << z << std::endl;
      app.getCameraPosition(&xget, &yget, &zget);
      std::cout << "Get position= " << xget << ", " << yget << ", " << zget << std::endl;
      getchar();
    }
    x = 0;
    y = 0;
    z = -10;
    for(int t = -10; t <= 10; t++)
    {
      Eigen::Quaterniond q;
      q = Eigen::AngleAxisd(t*M_PI/180., Eigen::Vector3d(1,0,0));
      app.main_light->setPosition(x,y,z);
      app.main_light->setDirection(-x,-y,-z);
      Mat im = vih.getVirtualImage(x, y, z, q.w(), q.x(), q.y(), q.z());
      std::cout << "Set quat= " << q.w() << ", " << q.x() << ", " << q.y() << ", " << q.z() << std::endl;
      app.getCameraOrientation(&qwget, &qxget, &qyget, &qzget);
      std::cout << "Get quat= " << qwget << ", " << qxget << ", " << qyget << ", " << qzget << std::endl;
      getchar();
    }
    
    for(int t = -10; t <= 10; t++)
    {
      Eigen::Quaterniond q;
      q = Eigen::AngleAxisd(t*M_PI/180., Eigen::Vector3d(0,1,0));
      app.main_light->setPosition(x,y,z);
      app.main_light->setDirection(-x,-y,-z);
      Mat im = vih.getVirtualImage(x, y, z, q.w(), q.x(), q.y(), q.z());
      std::cout << "Set quat= " << q.w() << ", " << q.x() << ", " << q.y() << ", " << q.z() << std::endl;
      app.getCameraOrientation(&qwget, &qxget, &qyget, &qzget);
      std::cout << "Get quat= " << qwget << ", " << qxget << ", " << qyget << ", " << qzget << std::endl;
      getchar();
    }

    for(int t = -10; t <= 10; t++)
    {
      Eigen::Quaterniond q;
      q = Eigen::AngleAxisd(t*M_PI/180., Eigen::Vector3d(0,0,1));
      app.main_light->setPosition(x,y,z);
      app.main_light->setDirection(-x,-y,-z);
      Mat im = vih.getVirtualImage(x, y, z, q.w(), q.x(), q.y(), q.z());
      std::cout << "Set quat= " << q.w() << ", " << q.x() << ", " << q.y() << ", " << q.z() << std::endl;
      app.getCameraOrientation(&qwget, &qxget, &qyget, &qzget);
      std::cout << "Get quat= " << qwget << ", " << qxget << ", " << qyget << ", " << qzget << std::endl;
      getchar();
    }
      //double qw, qx, qy, qz;
      //app.getCameraOrientation(&qw, &qx, &qy, &qz);

      //Mat pose = (Mat_<double>(4,4) << 1-2*qy*qy-2*qz*qz, 2*qx*qy -2*qz*qw, 2*qx*qz+2*qy*qw, x, 
       //                                2*qx*qy+2*qz*qw,1-2*qx*qx-2*qz*qz,2*qy*qz-2*qx*qw,    y,
       //                                2*qx*qz-2*qy*qw,2*qy*qz+2*qx*qw,1-2*qx*qx-2*qy*qy,    z,
      //                                 0,0,0,1);
      //im = vih.getVirtualImage(x, y, z, qw, qx, qy, qz);
      //Eigen::Matrix3f rot;
      //rot << 1-2*qy*qy-2*qz*qz, 2*qx*qy -2*qz*qw, 2*qx*qz+2*qy*qw, 
      //         2*qx*qy+2*qz*qw,1-2*qx*qx-2*qz*qz, 2*qy*qz-2*qx*qw,
      //         2*qx*qz-2*qy*qw,2*qy*qz+2*qx*qw,1-2*qx*qx-2*qy*qy;
      //Eigen::Quaternionf quat(rot);
      //std::cout << "orig quat=" << "(" << qw << "," << qx << "," << qy << "," << qz << ")" <<std::endl; 
      //std::cout << "eig quat=" << "(" << quat.w() << "," << quat.x() << "," << quat.y() << "," << quat.z() << ")" <<std::endl; 

    app.destroyScene();
  } catch(Ogre::Exception& e)  {
    std::cerr << "An exception has occurred: " <<
            e.getFullDescription().c_str() << std::endl;
  }
  return 0;
}

