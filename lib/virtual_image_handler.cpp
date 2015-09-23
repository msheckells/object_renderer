#include <stdio.h>

#include <opencv2/calib3d/calib3d.hpp>
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
  int width = imgWidth;
  int height = imgHeight;
  size_t bytesPerPixel = app->getBytesPerPixel();
  size_t bytesPerDepthPixel = app->getBytesPerDepthPixel();
  int_depth_data = app->mDepthBuffer;//new unsigned char[width*height*bytesPerDepthPixel];
  int_data = new unsigned char[width*height*bytesPerPixel];
}
//---------------------------------------------------------------------------
VirtualImageHandler::~VirtualImageHandler()
{
  delete[] int_depth_data;
  delete[] int_data;
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
void VirtualImageHandler::getVirtualImageAndDepthInternal(Mat& image, Mat& depth, double xp, 
  double yp, double zp, double w, double x, double y, double z)
{
  int width = imgWidth;
  int height = imgHeight;
  size_t bytesPerPixel = app->getBytesPerPixel();

  app->setCameraPosition(xp,yp,zp);
  app->setCameraOrientation(w,x,y,z);

  app->renderOnce();

  app->getRenderData(width, height, int_data);
  image = Mat(height, width, CV_8UC4, int_data);
  cvtColor(image, image, CV_BGRA2GRAY);

  //app->getDepthData(int_depth_data);
  depth = Mat(height, width, CV_32F, int_depth_data);
}

void VirtualImageHandler::getVirtualImageAndDepth(Mat& image, Mat& depth, double xp, double yp, 
  double zp, double w, double x, double y, double z)
{
  int width = imgWidth;
  int height = imgHeight;
  size_t bytesPerPixel = app->getBytesPerPixel();
  size_t bytesPerDepthPixel = app->getBytesPerDepthPixel();

  app->setCameraPosition(xp,yp,zp);
  app->setCameraOrientation(w,x,y,z);

  app->renderOnce();

  unsigned char* data = new unsigned char[width*height*bytesPerPixel];
  app->getRenderData(width, height, data);
  Mat im(height, width, CV_8UC4, data);
  cvtColor(im, image, CV_BGR2GRAY);
  delete[] data;

  unsigned char* depth_data = new unsigned char[width*height*bytesPerDepthPixel];
  app->getDepthData(depth_data);
  Mat dep(height, width, CV_32F, depth_data);
  depth = dep.clone();
  delete[] depth_data;
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
  Mat im_clone = im.clone();
  delete[] data;
  return im_clone;
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
  Mat im_clone = im.clone();
  delete[] data;
  return im_clone;
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

void VirtualImageHandler::filterKeypointsEpipolarConstraint(const std::vector<cv::Point2f>& pts1,
  const std::vector<cv::Point2f>& pts2, std::vector<cv::Point2f>& pts1_out, std::vector<cv::Point2f>& pts2_out)
{
  assert(pts1.size() == pts2.size());

  pts1_out.clear();
  pts2_out.clear();
  std::vector<unsigned char> status;
  cv::Mat fMat = findFundamentalMat(pts1, pts2, CV_FM_RANSAC, 4, .99, status);
  for(int i = 0; i < status.size(); i++)
  {
    if(status[i])
    {
      pts1_out.push_back(pts1[i]);
      pts2_out.push_back(pts2[i]);
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

void VirtualImageHandler::getFilteredFeatureMatches(Mat im1, Mat im2, std::vector<Point2f>& ps1_out, std::vector<Point2f>& ps2_out)
{
  ps1_out.clear();
  ps2_out.clear();

  std::vector<KeyPoint> kps1, kps2;
  std::vector < std::vector< DMatch > > matches;
  std::vector< DMatch > good_matches;
 
#ifdef OBJ_REND_USE_GPU
  gpu::GpuMat  desc_gpu1, desc_gpu2;
  getKeypointsAndDescriptors(im1, kps1, desc_gpu1);
  getKeypointsAndDescriptors(im2, kps2, desc_gpu2);

  gpu::BFMatcher_GPU matcher;
  matcher.knnMatch(desc_gpu1, desc_gpu2, matches, 2);
#else
  cv::Mat desc1, desc2;
  getKeypointsAndDescriptorsNoGpu(im1, kps1, desc1);
  getKeypointsAndDescriptorsNoGpu(im2, kps2, desc2);
  BFMatcher matcher;
  matcher.knnMatch(desc1, desc2, matches, 2); 
#endif

  Mat match_img, good_match_img;
  filterKeypointMatches(matches, good_matches, 0.8);
  /*
  drawMatches(im1, kps1, im2, kps2, matches, match_img);
  drawMatches(im1, kps1, im2, kps2, good_matches, good_match_img);
  imshow("All Matches", match_img);
  imshow("Good Matches", good_match_img);
  */
  std::cout << matches.size() << " matches" << std::endl;
  std::cout << good_matches.size() << " good matches" << std::endl;

  std::vector<Point2f> ps1, ps2;
  for(unsigned int i = 0; i < good_matches.size(); i++)
  {
    Point2f p1 = kps1[good_matches[i].queryIdx].pt;
    Point2f p2 = kps2[good_matches[i].trainIdx].pt;
    ps1.push_back(p1);
    ps2.push_back(p2);
  }

  std::vector<Point2f> ps1_filt, ps2_filt;
  filterKeypointsEpipolarConstraint(ps1, ps2, ps1_filt, ps2_filt);
  std::cout << ps1_filt.size() << " epi matches" << std::endl;

  /*
  std::vector<cv::KeyPoint> epi_kps1, epi_kps2;
  std::vector<std::vector<cv::DMatch>> epi_matches(ps1_filt.size());
  for(int i = 0; i < ps1_filt.size(); i++)
  {
    epi_kps1.push_back(KeyPoint(ps1_filt[i].x, ps1_filt[i].y, 1));
    epi_kps2.push_back(KeyPoint(ps2_filt[i].x, ps2_filt[i].y, 1));
    epi_matches[i].push_back(DMatch(i, i, 0));
  }
  Mat epi_match_img;
  drawMatches(im1, epi_kps1, im2, epi_kps2, epi_matches, epi_match_img);
  imshow("Epi Matches", epi_match_img);
  imshow("im1", im1);
  waitKey(0);
  */

  ps1_out = ps1_filt;
  ps2_out = ps2_filt;
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
