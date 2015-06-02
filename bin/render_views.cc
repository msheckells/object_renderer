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

  if (argc < 4)
  {
    std::cout << "Usage: " << argv[0] << " <model_file> <radius> <num_samples>" << std::endl;
    return -1;  
  }

  double sample_radius = std::stod(argv[2]);
  double num_samples = std::stoi(argv[3]);
  

  // Create application object
  CameraRenderApplication app;
  app.go();
  app.loadModel("model", std::string(argv[1])); 
  
  VirtualImageHandler vih(&app);
  try 
  {
    for(int i = 0; i < num_samples; i++)
    {
      double t = (double(rand())/RAND_MAX) * (2*M_PI);
      double p = (double(rand())/RAND_MAX) * (M_PI);
      double x = sample_radius*cos(t)*sin(p);
      double y = sample_radius*sin(t)*sin(p);
      double z = sample_radius*cos(p);
      app.main_light->setPosition(x,y,z);
      app.main_light->setDirection(-x,-y,-z);
      Mat im = vih.getVirtualImage(x, y, z, 0, 0, 0);
      Mat depth = vih.getVirtualDepth(x, y, z, 0, 0, 0);

      double qw, qx, qy, qz;
      app.getCameraOrientation(&qw, &qx, &qy, &qz);

      Mat pose = (Mat_<double>(4,4) << 1-2*qy*qy-2*qz*qz, 2*qx*qy -2*qz*qw, 2*qx*qz+2*qy*qw, x, 
                                       2*qx*qy+2*qz*qw,1-2*qx*qx-2*qz*qz,2*qy*qz-2*qx*qw,    y,
                                       2*qx*qz-2*qy*qw,2*qy*qz+2*qx*qw,1-2*qx*qx-2*qy*qy,    z,
                                       0,0,0,1);

      std::vector<KeyPoint> kps;
      Mat desc;

      vih.getKeypointsAndDescriptorsNoGpu(im, kps, desc);

      Mat kp_im;
      drawKeypoints(im, kps, kp_im);

      std::cout << "Image position: (" << x << ", " << y << ", " << z << ")" << std::endl;

      namedWindow( "Image Keypoints", WINDOW_AUTOSIZE );
      imshow("Image Keypoints", kp_im);
      waitKey(-1);
    
      FileStorage fs(std::string("keypoints") + std::to_string(i) + std::string(".xml"), FileStorage::WRITE);
      write( fs, "keypoints", kps );
      fs.release();   

      FileStorage fs_pose(std::string("pose") + std::to_string(i) + std::string(".xml"), FileStorage::WRITE);
      write( fs_pose, "pose", pose );
      fs_pose.release();   

      FileStorage fs_desc(std::string("descriptors") + std::to_string(i) + std::string(".xml"), FileStorage::WRITE);
      write( fs_desc, "descriptors", desc );
      fs_desc.release();   

      FileStorage fs_depth(std::string("depth") + std::to_string(i) + std::string(".xml"), FileStorage::WRITE);
      write( fs_depth, "depth", depth );
      fs_depth.release();   

      imwrite(std::string("keyframe") + std::to_string(i) + std::string(".jpg"), im );
      //app.saveDepthMap(std::string("depth") + std::to_string(i) + std::string(".png"));
    }
    app.destroyScene();
  } catch(Ogre::Exception& e)  {
    std::cerr << "An exception has occurred: " <<
            e.getFullDescription().c_str() << std::endl;
  }
  return 0;
}

