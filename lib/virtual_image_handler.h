
#ifndef __VIRTUAL_IMAGE_HANDLER_h_
#define __VIRTUAL_IMAGE_HANDLER_h_

#include "camera_render_application.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/gpu/gpu.hpp>
#include <opencv2/nonfree/gpu.hpp>


//---------------------------------------------------------------------------

class VirtualImageHandler 
{
public:
  VirtualImageHandler(CameraRenderApplication*);
  virtual ~VirtualImageHandler(void);

  int getImageHeight();
  int getImageWidth();
  cv::Mat getCameraIntrinsics();
  cv::Mat getVirtualDepth(double xp, double yp, double zp, double xl, double yl, double zl);
  cv::Mat getVirtualDepth(double xp, double yp, double zp, double w, double x, double y, double z);
  void getVirtualDepthNoShader(cv::Mat& depth,  double xp,
    double yp, double zp, double w, double x, double y, double z);
  cv::Mat getVirtualImage(double xp, double yp, double zp, double xl, double yl, double zl);    
  cv::Mat getVirtualImage(double xp, double yp, double zp, double w, double x, double y, double z);
  void getVirtualImageAndDepthInternal(cv::Mat& image, cv::Mat& depth, double xp, 
    double yp, double zp, double w, double x, double y, double z);
  void getVirtualImageAndDepth(cv::Mat& image, cv::Mat& depth, 
    double xp, double yp, double zp, double w, double x, double y, double z);
  unsigned char* getVirtualImage2(double xp, double yp, double zp, double w, double x, double y, double z);    
  cv::Mat getVirtualImage3(double xp, double yp, double zp, double w, double x, double y, double z);

  void getKeypointsAndDescriptorsNoGpu(cv::Mat& im, std::vector<cv::KeyPoint>& kps, cv::Mat& desc);
  void getKeypointsAndDescriptors(cv::Mat& im, std::vector<cv::KeyPoint>& kps, cv::gpu::GpuMat& desc_gpu);
  void filterKeypointMatches( std::vector < std::vector< cv::DMatch > >& matches, std::vector< cv::DMatch >& filtered_matches, double match_ratio);
  void filterKeypointsEpipolarConstraint(const std::vector<cv::Point2f>& pts1,
    const std::vector<cv::Point2f>& pts2, std::vector<cv::Point2f>& pts1_out, std::vector<cv::Point2f>& pts2_out);
  double getMeanKeypointError(std::vector<cv::KeyPoint>& kps1, std::vector<cv::KeyPoint>& kps2, std::vector< cv::DMatch >& matches);
  double getMeanKeypointError(cv::Mat im, std::vector<cv::KeyPoint>& kps_goal, cv::gpu::GpuMat& desc_gpu_goal, int* num_matches = 0);
  double getMeanKeypointErrorNoGpu(cv::Mat im, std::vector<cv::KeyPoint>& kps_goal, cv::Mat& desc_goal, int* num_matches = 0);
  void getFilteredFeatureMatches(cv::Mat im1, cv::Mat im2, std::vector<cv::Point2f>& ps1_out, std::vector<cv::Point2f>& ps2_out);



private:
  CameraRenderApplication* app;
  int imgHeight;
  int imgWidth;
  unsigned char* int_depth_data;
  unsigned char* int_data;
};


#endif

