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
#include <unordered_map>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <apriltag/image_u8.h>

#include "block_tracker.h"
#include "block_sensor.h"
#include "frame_annotator.h"

#include <argos3/core/utility/math/matrix/rotationmatrix3.h>
#include <argos3/core/utility/math/quaternion.h>
#include <argos3/core/utility/math/angles.h>

/****************************************/
/****************************************/

namespace std {
   template<>
   struct hash<std::list<STarget>::iterator> {
      size_t operator () (const std::list<STarget>::iterator& x) const {
         return std::hash<unsigned int>()(x->Id);
      }
   };
}

struct SLoadedImage {
   std::string FilePath;
   cv::Mat ImageData;
   std::chrono::time_point<std::chrono::steady_clock> Timestamp;
};

std::string strFileSuffix("baddetect");
std::string strFileExtension(".png");

int main(int n_arg_count, char* ppch_args[]) {
   std::chrono::time_point<std::chrono::steady_clock> tReferenceTime = std::chrono::steady_clock::now();
   std::vector<SLoadedImage> vecImages;

   if(n_arg_count > 1 && ppch_args[1][0] == '-') {    
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
            vecImages.push_back(SLoadedImage{strFileName, cImageData, tReferenceTime + std::chrono::milliseconds(nTimestamp)});
         }
         else {
            break;
         }
      } 
   }
   
   unsigned int unLoadedImagesCount = vecImages.size();

   if(unLoadedImagesCount != 0) {
      std::cerr << "Loaded " << unLoadedImagesCount << " images" << std::endl; 
      /* ensure images are in the correct order w.r.t. timestamps */
      std::sort(std::begin(vecImages),
                std::end(vecImages),
                [] (const SLoadedImage& s_lhs, const SLoadedImage& s_rhs) {
                  return s_lhs.Timestamp < s_rhs.Timestamp;
                }
      );
   }

   std::list<SBlock> lstDetectedBlocks;
   std::list<STarget> lstTrackedTargets;

   CBlockSensor* m_pcBlockSensor =
      new CBlockSensor;
   CBlockTracker* m_pcBlockTracker =
      new CBlockTracker(10u, 0.05f);
      
   cv::namedWindow("Output");
   cv::moveWindow("Output", 1975, 50);
 
   CFrameAnnotator cFrameAnnotator(m_pcBlockSensor->GetCameraMatrix(), m_pcBlockSensor->GetDistortionParameters());

   cv::VideoCapture cWebcam(0);
   cv::Mat cTmpImage;

   for(;;) {
      if(unLoadedImagesCount == 0) {
         vecImages.clear();
         vecImages.emplace_back();
         vecImages.back().FilePath = "webcam";
         cWebcam >> cTmpImage;
         vecImages.back().Timestamp = std::chrono::steady_clock::now();
         cv::cvtColor(cTmpImage, vecImages.back().ImageData, CV_BGR2GRAY);
      }

      for(SLoadedImage& s_loaded_image : vecImages) {
         /*
         if(unLoadedImagesCount != 0) {
            std::cerr << "processing: " << s_loaded_image.FilePath << std::endl;
         }
         */
         /* clear list of blocks from previous detection */                 
         lstDetectedBlocks.clear();

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

         /* Associate the known targets with the newly detected blocks */
         m_pcBlockTracker->AssociateAndTrackTargets(s_loaded_image.Timestamp, lstDetectedBlocks, lstTrackedTargets);
         
         double fTimestamp = std::chrono::duration<double, std::milli>(s_loaded_image.Timestamp - tReferenceTime).count();
         double m_fConnectivityThreshold = 0.055f * 1.25f; // 1.25 block distances
         /* define the target connectivity map */
         using TTargetIterator = std::list<STarget>::iterator;
         std::unordered_multimap<TTargetIterator, TTargetIterator> mapTargetConnectivity;
         /* build the target connectivity map */
         for(auto it_from_target = std::begin(lstTrackedTargets);
             it_from_target != std::end(lstTrackedTargets);
             it_from_target++) {
            for(auto it_to_target = std::begin(lstTrackedTargets);
                it_to_target != std::end(lstTrackedTargets);
                it_to_target++) {
               /* don't add transforms from a target to itself */               
               if(it_from_target != it_to_target) {
                  const SBlock& sFromObservation = it_from_target->Observations.front();
                  const SBlock& sToObservation = it_to_target->Observations.front();
                  if(argos::Distance(sFromObservation.Translation, sToObservation.Translation) < m_fConnectivityThreshold) {
                     /* populate the connectivity map */
                     mapTargetConnectivity.emplace(it_from_target, it_to_target);
                  }
               }
            }
         }
         /* define the root target as the target with the highest connectivity */
         auto itRootTarget = std::end(lstTrackedTargets);
         for(auto it_target = std::begin(lstTrackedTargets);
             it_target != std::end(lstTrackedTargets);
             it_target++) {
            if(mapTargetConnectivity.count(it_target) >= mapTargetConnectivity.count(itRootTarget)) {
               itRootTarget = it_target;
            }
         }

         if(itRootTarget != std::end(lstTrackedTargets)) {
            cFrameAnnotator.Annotate(*itRootTarget, cv::Scalar(0,255,255), "[R]");

            const SBlock& sRootObservation = (itRootTarget->PseudoObservations.size() != 0) ?
                itRootTarget->PseudoObservations.front() : itRootTarget->Observations.front();
         
            /* display the connectivity */
            for(auto it_target = std::begin(lstTrackedTargets);
                it_target != std::end(lstTrackedTargets);
                it_target++) {
         
               if(it_target == itRootTarget) {
                  continue;
               }

               const SBlock& sObservation = (it_target->PseudoObservations.size() != 0) ?
                  it_target->PseudoObservations.front() : it_target->Observations.front();

               /* calculate the relative transform */
               argos::CVector3 cTargetRelativeTranslation = sObservation.Translation;
               cTargetRelativeTranslation -= sRootObservation.Translation;
               cTargetRelativeTranslation.Rotate(sRootObservation.Rotation.Inverse());

               double fBlockWidth = 0.055f;

               std::ostringstream cCoords;

               cCoords << std::round(cTargetRelativeTranslation.GetX() / fBlockWidth) << ", ";
               cCoords << std::round(cTargetRelativeTranslation.GetY() / fBlockWidth) << ", ";
               cCoords << std::round(cTargetRelativeTranslation.GetZ() / fBlockWidth);

               //cCoords.str(std::to_string(it_target->Id));
               //std::cout << "Target #" << itRootTarget->Id << " => Target #" << it_target->Id << ": [" << cCoords.str() << "]" << std::endl;

               cv::Scalar cColor;
               switch(mapTargetConnectivity.count(it_target)) {
                  case 0:
                     cColor = cv::Scalar(255,0,0);
                     break;
                  case 1:
                     cColor = cv::Scalar(0,255,0);
                     break;
                  default:
                     cColor = cv::Scalar(0,0,255);
                     break;
               }
               std::ostringstream cText;
               cText << '[' << cCoords.str() << ']';
               cFrameAnnotator.Annotate(*it_target, cColor, cText.str());
            }
         }

         /* Create a color version of the image for annotation */
         cv::Mat cAnnotatedImage;
         cv::cvtColor(s_loaded_image.ImageData, cAnnotatedImage, CV_GRAY2BGR);
         cFrameAnnotator.WriteToFrame(cAnnotatedImage);
         cFrameAnnotator.Clear();

         //cv::Mat cOutputImg;
         //cv::flip(cAnnotatedImage, cOutputImg, 1);
         cv::imshow("Output", cAnnotatedImage);

/*
         std::ostringstream cFilePath;
         cFilePath << "/home/allsey87/Workspace/blocktracker-testbench/output/"
                   << "baddetect"
                   << std::setfill('0')
                   << std::setw(7)
                   << static_cast<int>(fTimestamp)
                   << ".png";
         cv::imwrite(cFilePath.str().c_str(), s_loaded_image.ImageData);
*/

         /* delay */
         if(cv::waitKey(10) == 'q') {
            break;
         }
      }
      if(unLoadedImagesCount != 0) {
         if(cv::waitKey(0) == 'q') {
            break;
         }
      }
   } 
}

