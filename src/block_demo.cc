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
   CBlockTracker* m_pcBlockTracker =
      new CBlockTracker(640u, 360u, 10u, 10u, 0.5f, 50.0f);

   std::list<SBlock> lstDetectedBlocks;
   std::list<STarget> lstTrackedTargets;
   
   std::string strInputPath("/home/allsey87/Workspace/blocktracker-testbench/sample/output_%05d.y.png");
       
   cv::namedWindow("Input Frame");
   cv::namedWindow("Block Detector Output");
   
   cv::VideoCapture* m_pcISSCaptureDevice = nullptr;
   
   unsigned int unBlockId = 0;
 
   do {
      delete m_pcISSCaptureDevice;
      m_pcISSCaptureDevice = new cv::VideoCapture(strInputPath.c_str());
   
      unsigned int unNumberOfFrames = m_pcISSCaptureDevice->get(CV_CAP_PROP_FRAME_COUNT);
   
      for(unsigned int unFrameIdx = 0; unFrameIdx < unNumberOfFrames; unFrameIdx++) {
         std::cerr << "processing frame " << unFrameIdx << std::endl;
         m_pcISSCaptureDevice->read(sCurrentFrame.U);
         cv::cvtColor(sCurrentFrame.U, sCurrentFrame.Y, CV_BGR2GRAY);
         
         cv::imshow("Input Frame", sCurrentFrame.U);

       
         //m_pcBlockSensor->SetCameraPosition();  
         m_pcBlockSensor->DetectBlocks(sCurrentFrame.Y, lstDetectedBlocks);

         /*
         unBlockId = 0;
         for(const SBlock& s_block : lstDetectedBlocks) {
            std::ostringstream cStream;
            cStream << '[' << unBlockId << ']';
            for(const STag& s_tag : s_block.Tags) {
               CFrameAnnotator::Annotate(sCurrentFrame.U, s_tag, cStream.str());
            }
            unBlockId++;
         }
         */

         // pass the time in miliseconds to track targets to allow for protectile based matching
         
         m_pcBlockTracker->AssociateAndTrackTargets(lstDetectedBlocks, lstTrackedTargets);

         for(const STarget& s_target : lstTrackedTargets) {
            ostringstream cText;
            cText << '[' << s_target.Id << ']';
            
         
            CFrameAnnotator::Annotate(sCurrentFrame.U,
                                      s_target,
                                      m_pcBlockSensor->GetCameraMatrix(),
                                      m_pcBlockSensor->GetDistortionParameters(),
                                      cText.str());
         }
         
         cv::imshow("Block Detector Output", sCurrentFrame.U);

         /*
         std::ostringstream cName;
         cName << "../output/" << std::setfill('0') << std::setw(3) << unFrameIdx << ".png";
         imwrite(cName.str().c_str(), sCurrentFrame.U);
         */
         
         /*
         // Cluster targets into structures
         CStructureDetectionAlgorithm::GenerateStructures();
         for(const CMicroRule& c_rule : vecMicroRules) {
            if(c_rule.Matches(set_of_detected_structures)) {
               m_pcActiveRule = &c_rule;
               break;
            }
         }
         */

         /* stream frame to host if connected */
         /*
         if(m_pcTCPImageSocket != nullptr) {
            *m_pcTCPImageSocket << sCurrentFrame.Y;
         }
         */
         cv::waitKey(1);
      }
   } while(cv::waitKey(0) != 'q');
}

