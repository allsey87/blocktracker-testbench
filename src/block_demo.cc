#include <cstring>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <chrono>

#include <error.h>
#include <signal.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <unistd.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <apriltag/image_u8.h>

#include "block_tracker.h"
#include "block_sensor.h"
#include "frame_annotator.h"

/****************************************/
/****************************************/

struct SLoadedImage {
   std::string FilePath;
   cv::Mat ImageData;
   std::chrono::time_point<std::chrono::steady_clock> Timestamp;
};

std::string strFileSuffix("test_");
std::string strFileExtension(".png");

int main(int n_arg_count, char* ppch_args[]) {
   std::chrono::time_point<std::chrono::steady_clock> tReferenceTime = std::chrono::steady_clock::now();
   std::vector<SLoadedImage> vecLoadedImages;

   /* read filenames from std::cin */
   std::string strFileName;
   cv::Mat cImageData;
   int nTimestamp;
   for(;;) {
      std::cin >> strFileName;
      if(std::cin.good()) {
         cImageData = cv::imread(strFileName.c_str(), CV_LOAD_IMAGE_GRAYSCALE);
         if(cImageData.data == nullptr) {
            std::cerr << "could not load: " << strFileName << std::endl;
            continue;
         }
         std::string::size_type unPosStart = strFileName.find(strFileSuffix) + strFileSuffix.size();
         std::string::size_type unPosEnd = strFileName.find(strFileExtension);
         if(unPosStart == std::string::npos || unPosEnd == std::string::npos) {
            std::cerr << "could not find suffix/extension in file name: " << strFileName << std::endl;
            continue;
         }
         std::string strTimestamp = strFileName.substr(unPosStart, unPosEnd - unPosStart);
         try {
            nTimestamp = std::stoi(strTimestamp);
         }
         catch(const std::exception& ex) {
            std::cerr << "could not parse timestamp: " << strTimestamp << " from " << strFileName << std::endl;
            continue;
         }
         vecLoadedImages.push_back(SLoadedImage{strFileName, cImageData, tReferenceTime + std::chrono::milliseconds(nTimestamp)});
      }
      else {
         break;
      }
   }
   /* output the number of loaded images */
   std::cerr << "Loaded " << vecLoadedImages.size() << " images" << std::endl;
   /* ensure images are in the correct order w.r.t. timestamps */
   std::sort(std::begin(vecLoadedImages),
             std::end(vecLoadedImages),
             [] (const SLoadedImage& s_lhs, const SLoadedImage& s_rhs) {
               return s_lhs.Timestamp < s_rhs.Timestamp;
             }
   );

   // timestamp in seconds as a float
   //std::chrono::duration<float>(s_loaded_image.Timestamp - tReferenceTime).count()

   std::list<SBlock> lstDetectedBlocks;
   std::list<STarget> lstTrackedTargets;

   CBlockSensor* m_pcBlockSensor =
      new CBlockSensor;
   CBlockTracker* m_pcBlockTracker =
      new CBlockTracker(640u, 360u, 3u, 0.5f);
      
   cv::namedWindow("Input Frame");
   cv::namedWindow("Block Detector Output");
   cv::moveWindow("Input Frame", 1975, 50);
   cv::moveWindow("Block Detector Output", 1975, 500);
 
   for(;;) {
      for(SLoadedImage& s_loaded_image : vecLoadedImages) {
         std::cerr << "processing: " << s_loaded_image.FilePath << std::endl;
         
         cv::imshow("Input Frame", s_loaded_image.ImageData);
         
         lstDetectedBlocks.clear();
         std::chrono::time_point<std::chrono::steady_clock> tDetectionTime = s_loaded_image.Timestamp;

         /* convert image to apriltags format */
         image_u8_t* ptImageY = image_u8_create(s_loaded_image.ImageData.cols, s_loaded_image.ImageData.rows);
         for (unsigned int un_row = 0; un_row < ptImageY->height; un_row++) {
            std::memcpy(&ptImageY->buf[un_row * ptImageY->stride],
            s_loaded_image.ImageData.row(un_row).data,
            ptImageY->width);
         }         
         /* Detect blocks */
         m_pcBlockSensor->DetectBlocks(ptImageY, ptImageY, ptImageY, lstDetectedBlocks);
         /* Deallocate the apriltags image */
         image_u8_destroy(ptImageY);

         cv::Mat cAnnotatedImage;
         cv::cvtColor(s_loaded_image.ImageData, cAnnotatedImage, CV_GRAY2BGR);
         
         /*
         unsigned int unBlockId = 0;
         for(const SBlock& s_block : lstDetectedBlocks) {
            std::ostringstream cStream;
            cStream << '[' << unBlockId << ']';
            for(const STag& s_tag : s_block.Tags) {
               CFrameAnnotator::Annotate(cAnnotatedImage, s_tag, cStream.str());
            }
            unBlockId++;
         }
         */

         // pass the time in miliseconds to track targets to allow for protectile based matching

         std::cerr << "lstDetectedBlocks.size() = " << lstDetectedBlocks.size() << std::endl;
         m_pcBlockTracker->AssociateAndTrackTargets(s_loaded_image.Timestamp, lstDetectedBlocks, lstTrackedTargets);
         std::cerr << "lstTrackedTargets.size() = " << lstTrackedTargets.size() << std::endl;

         for(const STarget& s_target : lstTrackedTargets) {
            std::ostringstream cText;
            cText << '[' << s_target.Id << ']';
            CFrameAnnotator::Annotate(cAnnotatedImage,
                                      s_target,
                                      m_pcBlockSensor->GetCameraMatrix(),
                                      m_pcBlockSensor->GetDistortionParameters(),
                                      cText.str(),
                                      tReferenceTime);
         }
         cv::imshow("Block Detector Output", cAnnotatedImage);

         float fTimestamp = std::chrono::duration<float, std::milli>(s_loaded_image.Timestamp - tReferenceTime).count();
         std::ostringstream cFilePath;

         cFilePath << "/home/allsey87/Workspace/blocktracker-testbench/output/"
                   << "out"
                   << std::setfill('0')
                   << std::setw(7)
                   << static_cast<int>(fTimestamp)
                   << ".png";

         cv::imwrite(cFilePath.str().c_str(), cAnnotatedImage);
        
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

         /* delay */
         if(cv::waitKey(0) == 'q') {
            break;
         }
      }
      if(cv::waitKey(0) == 'q') {
         break;
      }
   } 
}

