#include <stdio.h>

#include <opencv2/nonfree/features2d.hpp>
#include "virtual_image_handler.h"

using namespace cv;

//---------------------------------------------------------------------------
VirtualImageHandler::VirtualImageHandler(CameraRenderApplication* cra) : 
  app(cra),
  imgHeight(app->getWindowHeight()),
  imgWidth(app->getWindowWidth())
{
  std::cout << "Image Size: " << imgWidth << "x" <<imgHeight << std::endl;
}
//---------------------------------------------------------------------------
VirtualImageHandler::~VirtualImageHandler()
{
}
  
int VirtualImageHandler::getImageHeight()
{
  return imgHeight;
}

int VirtualImageHandler::getImageWidth()
{
  return imgWidth;
}

unsigned char* VirtualImageHandler::getVirtualImage2(double xp, double yp, double zp, double w, double x, double y, double z)
{
  int width = imgWidth;
  int height = imgHeight;
  size_t bytesPerPixel = app->getBytesPerPixel();

  app->setCameraPosition(xp,yp,zp);
  app->setCameraOrientation(w,x,y,z);

  app->renderOnce();

  unsigned char* data = new unsigned char[width*height*bytesPerPixel];
  app->getRenderData(width, height, data);
  return data;
}

Mat VirtualImageHandler::getVirtualImage(double xp, double yp, double zp, double w, double x, double y, double z)
{
  int width = imgWidth;
  int height = imgHeight;
  size_t bytesPerPixel = app->getBytesPerPixel();

  app->setCameraPosition(xp,yp,zp);
  app->setCameraOrientation(w,x,y,z);

  app->renderOnce();

  unsigned char* data = new unsigned char[width*height*bytesPerPixel];
  app->getRenderData(width, height, data);
  Mat im(height, width, CV_8UC4, data);
  cvtColor(im, im, CV_BGRA2GRAY);
  return im;
}

Mat VirtualImageHandler::getVirtualDepth(double xp, double yp, double zp, double w, double x, double y, double z)
{
  int width = imgWidth;
  int height = imgHeight;
  size_t bytesPerPixel = app->getBytesPerDepthPixel();

  app->setCameraPosition(xp,yp,zp);
  app->setCameraOrientation(w,x,y,z);

  app->renderOnce();

  unsigned char* data = new unsigned char[width*height*bytesPerPixel];
  app->getDepthData(data);
  Mat im(height, width, CV_32F, data);
  return im;
}
Mat VirtualImageHandler::getVirtualDepth(double xp, double yp, double zp, double xl, double yl, double zl)
{
  int width = imgWidth;
  int height = imgHeight;
  size_t bytesPerPixel = app->getBytesPerDepthPixel();

  app->setCameraPosition(xp,yp,zp);
  app->setCameraLookAt(xl,yl,zl);

  app->renderOnce();

  unsigned char* data = new unsigned char[width*height*bytesPerPixel];
  app->getDepthData(data);
  Mat im(height, width, CV_32F, data);
  
  for(int i = 0; i < height; i++)
  {
    for(int j = 0; j < width; j++)
    {
      if(im.at<float>(i, j) == 0)
      {
        im.at<float>(i, j) = -1;
      }
    }
  }
  return im;
}


Mat VirtualImageHandler::getCameraIntrinsics()
{
  double fx, fy, cx, cy;
  app->getCameraIntrinsics(&fx, &fy, &cx, &cy);
  Mat intrinsics;
  intrinsics = (Mat_<double>(3,3) << fx,  0, cx,
                                      0, fy, cy,
                                      0,  0, 1);
  return intrinsics; 
}

Mat VirtualImageHandler::getVirtualImage(double xp, double yp, double zp, double xl, double yl, double zl)
{
  int width = imgWidth;
  int height = imgHeight;
  size_t bytesPerPixel = app->getBytesPerPixel();

  app->setCameraPosition(xp,yp,zp);
  app->setCameraLookAt(xl,yl,zl);

  app->renderOnce();

  unsigned char* data = new unsigned char[width*height*bytesPerPixel];
  app->getRenderData(width, height, data);
  Mat im(height, width, CV_8UC4, data);
  cvtColor(im, im, CV_BGR2GRAY);
  return im;
}
void VirtualImageHandler::getKeypointsAndDescriptors(Mat& im, std::vector<KeyPoint>& kps, gpu::GpuMat& desc_gpu)
{
  gpu::GpuMat kps_gpu, im_gpu(im);

  gpu::SURF_GPU surf_gpu;
  surf_gpu(im_gpu, gpu::GpuMat(), kps_gpu, desc_gpu);
  surf_gpu.downloadKeypoints(kps_gpu, kps);
}

void VirtualImageHandler::getKeypointsAndDescriptorsNoGpu(Mat& im, std::vector<KeyPoint>& kps, Mat& desc)
{
  SURF surf;
  surf(im, Mat(), kps, desc);
}

void VirtualImageHandler::filterKeypointMatches( std::vector < std::vector< DMatch > >& matches, std::vector< DMatch >& filtered_matches, double match_ratio)
{
  for(unsigned int i = 0; i < matches.size(); i++)
  {
    if(matches[i][0].distance < match_ratio*matches[i][1].distance)
    {
      filtered_matches.push_back(matches[i][0]);
    }
  }
}

double VirtualImageHandler::getMeanKeypointError(std::vector<KeyPoint>& kps1, std::vector<KeyPoint>& kps2, std::vector< DMatch >& matches)
{
  double error = 0.;

  for(unsigned int i = 0; i < matches.size(); i++)
  {
    Point2f p1 = kps1[matches[i].queryIdx].pt;
    Point2f p2 = kps2[matches[i].trainIdx].pt;
    double n = norm(p1-p2);
    error += (n*n)/matches.size();
  }

  return error;
}

double VirtualImageHandler::getMeanKeypointError(Mat im, std::vector<KeyPoint>& kps_goal, gpu::GpuMat& desc_gpu_goal, int* num_matches)
{
  std::vector<KeyPoint> kps;
  gpu::GpuMat  desc_gpu;
  std::vector < std::vector< DMatch > > matches;
  std::vector< DMatch > good_matches;
 
  getKeypointsAndDescriptors(im, kps, desc_gpu);

  if(kps.size() <= 0 || kps_goal.size() <= 0)
  {
    if(num_matches)
    {
      num_matches = 0;
    }
    return 0;
  }

  gpu::BFMatcher_GPU matcher;
  matcher.knnMatch(desc_gpu, desc_gpu_goal, matches, 2);
  filterKeypointMatches(matches, good_matches, 0.5);

  if(num_matches)
  {
    *num_matches = good_matches.size();
  }

  return getMeanKeypointError(kps, kps_goal, good_matches);
}

double VirtualImageHandler::getMeanKeypointErrorNoGpu(Mat im, std::vector<KeyPoint>& kps_goal, Mat& desc_goal, int* num_matches)
{
  std::vector<KeyPoint> kps;
  Mat  desc;
  std::vector < std::vector< DMatch > > matches;
  std::vector< DMatch > good_matches;
 
  getKeypointsAndDescriptorsNoGpu(im, kps, desc);

  if(kps.size() <= 0 || kps_goal.size() <= 0)
  {
    if(num_matches)
    {
      num_matches = 0;
    }
    return 0;
  }

  BFMatcher matcher;
  matcher.knnMatch(desc, desc_goal, matches, 2);
  filterKeypointMatches(matches, good_matches, 0.65);

  if(num_matches)
  {
    *num_matches = good_matches.size();
  }

  return getMeanKeypointError(kps, kps_goal, good_matches);
}
