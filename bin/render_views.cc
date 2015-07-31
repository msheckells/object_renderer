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

CameraRenderApplication* app;
VirtualImageHandler* vih;

void saveView(double x, double y, double z, std::string output_dir, int file_num)
{
  try
  {
    app->main_light->setPosition(x,y,z);
    app->main_light->setDirection(-x,-y,-z);
    Mat im = vih->getVirtualImage(x, y, z, 0, 0, 0);
    Mat depth = vih->getVirtualDepth(x, y, z, 0, 0, 0);
  
    double qw, qx, qy, qz;
    app->getCameraOrientation(&qw, &qx, &qy, &qz);
  
    Mat pose = (Mat_<double>(4,4) << 1-2*qy*qy-2*qz*qz, 2*qx*qy -2*qz*qw, 2*qx*qz+2*qy*qw, x, 
                                     2*qx*qy+2*qz*qw,1-2*qx*qx-2*qz*qz,2*qy*qz-2*qx*qw,    y,
                                     2*qx*qz-2*qy*qw,2*qy*qz+2*qx*qw,1-2*qx*qx-2*qy*qy,    z,
                                     0,0,0,1);
    //im = vih.getVirtualImage(x, y, z, qw, qx, qy, qz);
    //Eigen::Matrix3f rot;
    //rot << 1-2*qy*qy-2*qz*qz, 2*qx*qy -2*qz*qw, 2*qx*qz+2*qy*qw, 
    //         2*qx*qy+2*qz*qw,1-2*qx*qx-2*qz*qz, 2*qy*qz-2*qx*qw,
    //         2*qx*qz-2*qy*qw,2*qy*qz+2*qx*qw,1-2*qx*qx-2*qy*qy;
    //Eigen::Quaternionf quat(rot);
    //std::cout << "orig quat=" << "(" << qw << "," << qx << "," << qy << "," << qz << ")" <<std::endl; 
    //std::cout << "eig quat=" << "(" << quat.w() << "," << quat.x() << "," << quat.y() << "," << quat.z() << ")" <<std::endl; 

    std::vector<KeyPoint> kps;
    Mat desc;

    vih->getKeypointsAndDescriptorsNoGpu(im, kps, desc);

    Mat kp_im;
    drawKeypoints(im, kps, kp_im);
  
    std::cout << "Image position: (" << x << ", " << y << ", " << z << ")" << std::endl;
  
    imshow("Image Keypoints", kp_im);
    waitKey(1);
     
    std::stringstream ss;
    ss << output_dir << "/keypoints" << std::setw(3) << std::setfill('0') << file_num << ".xml";
    FileStorage fs(ss.str(), FileStorage::WRITE);
    write( fs, "keypoints", kps );
    fs.release();   
  
    ss.str(std::string());
    ss << output_dir << "/pose" << std::setw(3) << std::setfill('0') << file_num << ".xml";
    FileStorage fs_pose(ss.str(), FileStorage::WRITE);
    write( fs_pose, "pose", pose );
    fs_pose.release();   
  
    ss.str(std::string());
    ss << output_dir << "/descriptors" << std::setw(3) << std::setfill('0') << file_num << ".xml";
    FileStorage fs_desc(ss.str(), FileStorage::WRITE);
    write( fs_desc, "descriptors", desc );
    fs_desc.release();   
 
    ss.str(std::string());
    ss << output_dir << "/depth" << std::setw(3) << std::setfill('0') << file_num << ".xml";
    FileStorage fs_depth(ss.str(), FileStorage::WRITE);
    write( fs_depth, "depth", depth );
    fs_depth.release();   

    ss.str(std::string());
    ss << output_dir << "/keyframe" << std::setw(3) << std::setfill('0') << file_num << ".jpg";
    imwrite(ss.str(), im );
  } catch(Ogre::Exception& e)  {
    std::cerr << "An exception has occurred: " <<
            e.getFullDescription().c_str() << std::endl;
  }

}

int main(int argc, char *argv[])
{

  if (argc < 5)
  {
    std::cout << "Usage: " << argv[0] << " <output_dir> <model_file> <radius> <num_samples> [<start_num>]" << std::endl;
    return -1;  
  }

  std::string output_dir(argv[1]);
  double sample_radius = std::stod(argv[3]);
  double num_samples = std::stoi(argv[4]);
  int start_num;
  if(argc >= 6)
  {
    start_num = std::stoi(argv[5]);
  }
  else
  {
    start_num = 0;
  }

  // Create application object
  app = new CameraRenderApplication("../../cfg/");
  app->go();
  app->loadModel("model", std::string(argv[2])); 
  
  vih = new VirtualImageHandler(app);
  namedWindow( "Image Keypoints", WINDOW_AUTOSIZE );

  std::cout << "Intrinsics:" << std::endl << vih->getCameraIntrinsics() << std::endl;
  FileStorage fs_intrinsics(output_dir + std::string("/intrinsics.xml"), FileStorage::WRITE);
  write( fs_intrinsics, "intrinsics",  vih->getCameraIntrinsics());
  fs_intrinsics.release();   

  if(num_samples == -1)
  {
    // Do 6 views
    saveView(-sample_radius, 0, 0, output_dir, start_num+0);
    saveView( sample_radius, 0, 0, output_dir, start_num+1);
    saveView(0, -sample_radius, 0, output_dir, start_num+2);
    saveView(0,  sample_radius, 0, output_dir, start_num+3);
    saveView(0, 0, -sample_radius, output_dir, start_num+4);
    saveView(0, 0,  sample_radius, output_dir, start_num+5);
  }
  else
  {
    for(int i = 0; i < num_samples; i++)
    {
      double t = (double(rand())/RAND_MAX) * (2*M_PI);
      double p = (double(rand())/RAND_MAX) * (M_PI);
      double x = sample_radius*cos(t)*sin(p);
      double y = sample_radius*sin(t)*sin(p);
      double z = sample_radius*cos(p);
      saveView(x, y, z, output_dir, start_num+i);
      getchar();
    }
  }
  return 0;
}

