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
   cv::namedWindow("Block Detector Output");
     
   unsigned int unBlockId = 0;
 
   cv::VideoCapture* m_pcISSCaptureDevice = new cv::VideoCapture(strInputPath.c_str());

   unsigned int unNumberOfFrames = m_pcISSCaptureDevice->get(CV_CAP_PROP_FRAME_COUNT);

   m_pcISSCaptureDevice->read(sCurrentFrame.U);
   cv::cvtColor(sCurrentFrame.U, sCurrentFrame.Y, CV_BGR2GRAY);
   
   //cv::imshow("Input Frame", sCurrentFrame.U);

   m_pcBlockSensor->DetectBlocks(sCurrentFrame.Y, lstDetectedBlocks);
   
   for(SBlock& s_block : lstDetectedBlocks) {
      std::cerr << "Rotation Vector: " << std::endl
                << s_block.RotationVector << std::endl;             
   }
   
   for(SBlock& s_block : lstDetectedBlocks) {
      std::cerr << "Yaw, Pitch, Roll: " << std::endl
                << s_block.Yaw << ", " << s_block.Pitch << ", " << s_block.Roll << std::endl;
   }
   
   unsigned int i = 0;
      
   for(SBlock& s_block : lstDetectedBlocks) {
      CFrameAnnotator::Annotate(sCurrentFrame.U, s_block.Tags[0], std::to_string(i));
      i++;
   }
   
   cv::imshow("Block Detector Output", sCurrentFrame.U);

   cv::waitKey(0);
}

