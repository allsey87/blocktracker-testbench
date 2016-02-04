#include <cstring>
#include <iostream>
#include <iomanip>
#include <fstream>

#include <error.h>
#include <signal.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <unistd.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "block_tracker.h"
#include "block_sensor.h"
#include "frame_annotator.h"

/****************************************/
/****************************************/


int main(int n_arg_count, char* ppch_args[]) {
   struct {
      cv::Mat Y;
      cv::Mat U;
      cv::Mat V;
   } sCurrentFrame;

   CBlockSensor* m_pcBlockSensor =
      new CBlockSensor;

   std::list<SBlock> lstDetectedBlocks;
   
   std::string strInputPath("/home/allsey87/Workspace/blocktracker-testbench/block.png");
       
   //cv::namedWindow("Input Frame");
   //cv::namedWindow("Block Detector Output");
     
   unsigned int unBlockId = 0;
 
   //cv::VideoCapture* m_pcISSCaptureDevice = new cv::VideoCapture(strInputPath.c_str());
   cv::VideoCapture* m_pcISSCaptureDevice = new cv::VideoCapture(0);

   //unsigned int unNumberOfFrames = m_pcISSCaptureDevice->get(CV_CAP_PROP_FRAME_COUNT);


   while(m_pcISSCaptureDevice->read(sCurrentFrame.U)) {

      cv::cvtColor(sCurrentFrame.U, sCurrentFrame.Y, CV_BGR2GRAY);
      
      //cv::imshow("Input Frame", sCurrentFrame.U);

      m_pcBlockSensor->DetectBlocks(sCurrentFrame.Y, sCurrentFrame.U, sCurrentFrame.V, lstDetectedBlocks);
      
      
      /*
      for(SBlock& s_block : lstDetectedBlocks) {
         std::cerr << "Rotation Vector: " << std::endl
                   << s_block.RotationVector << std::endl;             
      }
      
      for(SBlock& s_block : lstDetectedBlocks) {
         std::cerr << "Yaw, Pitch, Roll: " << std::endl
                   << s_block.Yaw << ", " << s_block.Pitch << ", " << s_block.Roll << std::endl;
      }
      */
      
      /*      
      unsigned int i = 0;
         
      for(SBlock& s_block : lstDetectedBlocks) {
         CFrameAnnotator::Annotate(sCurrentFrame.U, s_block.Tags[0], std::to_string(i));
         i++;
      }
      */
      
      std::vector<cv::Point3f> vecGCSAxesPoints = {
         cv::Point3f(0, 0, 0),
         cv::Point3f(0.05, 0, 0),
         cv::Point3f(0, 0.05, 0),
         cv::Point3f(0, 0, 0.05),
      };
      std::vector<cv::Point2f> vecGCSAxesPixels;
      cv::projectPoints(vecGCSAxesPoints,
                        cv::Matx31f(0, 0, 0),
                        cv::Matx31f(0, 0, 0),
                        m_pcBlockSensor->GetCameraMatrix(),
                        m_pcBlockSensor->GetDistortionParameters(),
                        vecGCSAxesPixels);
                             
      cv::line(sCurrentFrame.U, vecGCSAxesPixels[0], vecGCSAxesPixels[1], cv::Scalar(0,0,255), 2);
      cv::line(sCurrentFrame.U, vecGCSAxesPixels[0], vecGCSAxesPixels[2], cv::Scalar(0,255,0), 2);
      cv::line(sCurrentFrame.U, vecGCSAxesPixels[0], vecGCSAxesPixels[3], cv::Scalar(255,0,0), 2);

      cv::imshow("Block Detector Output", sCurrentFrame.U);
      if(cv::waitKey(1) != -1) {
         return 0;
      }
   }
   cv::waitKey(0);
   return 0;
}

