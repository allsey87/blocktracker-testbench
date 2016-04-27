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

std::ostream& operator<<(std::ostream& c_stream, const SBlock& s_block) {
   auto tInitFlags = c_stream.flags();
   auto tInitPrec = c_stream.precision();
   c_stream.setf(std::ios::fixed);
   c_stream.precision(3);
   c_stream << "T["
            << s_block.Translation.X << ", "
            << s_block.Translation.Y << ", "
            << s_block.Translation.Z << "]"
            << '\t';
   c_stream << "R["
            << (s_block.Rotation.Z * 180.0f) / M_PI << ", "
            << (s_block.Rotation.Y * 180.0f) / M_PI << ", "
            << (s_block.Rotation.X * 180.0f) / M_PI << "]";
   c_stream.flags(tInitFlags);
   c_stream.precision(tInitPrec);
   return c_stream;
}

std::ostream& print_matrix(std::ostream& c_stream, const cv::Mat& c_cv_matrix) {
   auto tInitFlags = c_stream.flags();
   auto tInitPrec = c_stream.precision();
   
   c_stream.setf(std::ios::fixed);
   c_stream.precision(2);

   for(int i = 0; i < c_cv_matrix.size().height; i++) {
      c_stream << "[";
      for(int j = 0; j < c_cv_matrix.size().width; j++) {
         c_stream << c_cv_matrix.at<double>(i,j);
         if(j != c_cv_matrix.size().width - 1) {
            c_stream << ", ";
         }
         else {
            c_stream << "]" << std::endl;
         }
      }
   }

   c_stream.flags(tInitFlags);
   c_stream.precision(tInitPrec);   

   return c_stream;
}


struct SLoadedImage {
   std::string FilePath;
   cv::Mat ImageData;
   std::chrono::time_point<std::chrono::steady_clock> Timestamp;
};

std::string strFileSuffix("test_");
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
      new CBlockTracker(640u, 360u, 3u, 0.05f);
      
   cv::namedWindow("Output");
   cv::moveWindow("Output", 1975, 50);
 
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
         if(unLoadedImagesCount != 0) {
            std::cerr << "processing: " << s_loaded_image.FilePath << std::endl;
         }
                 
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
         cv::Mat temp(ptImageY->height, ptImageY->width, CV_8UC1, ptImageY->buf, ptImageY->stride);
         cv::Mat cAnnotatedImage;
         cv::cvtColor(temp, cAnnotatedImage, CV_GRAY2BGR);

         image_u8_destroy(ptImageY);

         //cv::Mat cAnnotatedImage;
         //cv::cvtColor(s_loaded_image.ImageData, cAnnotatedImage, CV_GRAY2BGR);
         
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
         for(const SBlock& s_block : lstDetectedBlocks)
            CFrameAnnotator::Annotate(cAnnotatedImage,
                                      s_block,
                                      m_pcBlockSensor->GetCameraMatrix(),
                                      m_pcBlockSensor->GetDistortionParameters());



         // pass the time in miliseconds to track targets to allow for protectile based matching
         m_pcBlockTracker->AssociateAndTrackTargets(s_loaded_image.Timestamp, lstDetectedBlocks, lstTrackedTargets);
         /*
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
         */
         double fTimestamp = std::chrono::duration<double, std::milli>(s_loaded_image.Timestamp - tReferenceTime).count();

         double m_fConnectivityThreshold = 0.055f * 1.25f; // 1.25 block distances

         using TTargetIterator = std::list<STarget>::iterator;

         struct STransform {
            cv::Mat Rotation;
            cv::Mat Translation;
            TTargetIterator Other;
         };


         std::unordered_multimap<TTargetIterator, TTargetIterator> mapTargetConnectivity;

         /* build the structure map */
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
                  if(std::sqrt(std::pow(sFromObservation.Translation.X - sToObservation.Translation.X, 2) +
                               std::pow(sFromObservation.Translation.Y - sToObservation.Translation.Y, 2) +
                               std::pow(sFromObservation.Translation.Z - sToObservation.Translation.Z, 2)) < m_fConnectivityThreshold) {
                     /* populate the connectivity map */
                     mapTargetConnectivity.emplace(it_from_target, it_to_target);
                  }  
               }
            }
         }

         auto itRootTarget = std::end(lstTrackedTargets);

         for(auto it_target = std::begin(lstTrackedTargets);
             it_target != std::end(lstTrackedTargets);
             it_target++) {
            if(mapTargetConnectivity.count(it_target) >= mapTargetConnectivity.count(itRootTarget)) {
               itRootTarget = it_target;
            }
         }

         if(itRootTarget != std::end(lstTrackedTargets)) {
            CFrameAnnotator::Annotate(cAnnotatedImage,
                                         *itRootTarget,
                                         m_pcBlockSensor->GetCameraMatrix(),
                                         m_pcBlockSensor->GetDistortionParameters(),
                                         "[R]",
                                         cv::Scalar(0,255,255));
         
            /* build the transformation matrix */
            cv::Mat cRootTargetTransform = cv::Mat::zeros(4, 4, CV_64F);

            const SBlock& sRootTargetBlock = itRootTarget->Observations.front();

            cRootTargetTransform.at<double>(0, 3) = sRootTargetBlock.Translation.X;
            cRootTargetTransform.at<double>(1, 3) = sRootTargetBlock.Translation.Y;
            cRootTargetTransform.at<double>(2, 3) = sRootTargetBlock.Translation.Z;
            cRootTargetTransform.at<double>(3, 3) = 1.0f;

            argos::CRadians pcRootTargetEulerAngles[] = {
               argos::CRadians(sRootTargetBlock.Rotation.Z),
               argos::CRadians(sRootTargetBlock.Rotation.Y),
               argos::CRadians(sRootTargetBlock.Rotation.X),
            };

            argos::CQuaternion cRootTargetRotationQuaternion;
            cRootTargetRotationQuaternion.FromEulerAngles(pcRootTargetEulerAngles[0],
                                                          pcRootTargetEulerAngles[1],
                                                          pcRootTargetEulerAngles[2]);
            argos::CRotationMatrix3 cRootTargetRotationMatrix(cRootTargetRotationQuaternion);

            /* copy rotation matrix values */
            cv::Mat(3, 3, CV_64F, &cRootTargetRotationMatrix(0,0)).copyTo(cRootTargetTransform(cv::Rect(0,0,3,3)));

            // For each other target, rebuild the cv translation and rotation vectors
            // use cv::RTCompose to add the transforms of the other targets to the inverse of the root target
            // print out coordinates x,y,z - hopefully, they should be combinations of +/-1

            /* 

            cv::Mat cRootTargetInvRotationVector, cRootTargetInvTranslationVector;
            cv::Rodrigues(cRootTargetInvTransform(cv::Rect(0,0,3,3)), cRootTargetInvRotationVector);
            cRootTargetInvTranslationVector = cRootTargetInvTransform(cv::Rect(3,0,1,3));

            */
            
            /* display the connectivity */
            for(auto it_target = std::begin(lstTrackedTargets);
                it_target != std::end(lstTrackedTargets);
                it_target++) {
         
               if(it_target == itRootTarget) {
                  continue;
               }

               cv::Mat cTargetTransform = cv::Mat::zeros(4, 4, CV_64F);

               const SBlock& sTargetBlock = it_target->Observations.front();

               cTargetTransform.at<double>(0, 3) = sTargetBlock.Translation.X;
               cTargetTransform.at<double>(1, 3) = sTargetBlock.Translation.Y;
               cTargetTransform.at<double>(2, 3) = sTargetBlock.Translation.Z;
               cTargetTransform.at<double>(3, 3) = 1.0f;

               argos::CRadians pcTargetEulerAngles[] = {
                  argos::CRadians(sTargetBlock.Rotation.Z),
                  argos::CRadians(sTargetBlock.Rotation.Y),
                  argos::CRadians(sTargetBlock.Rotation.X),
               };

               argos::CQuaternion cTargetRotationQuaternion;
               cTargetRotationQuaternion.FromEulerAngles(pcTargetEulerAngles[0],
                                                         pcTargetEulerAngles[1],
                                                         pcTargetEulerAngles[2]);
               argos::CRotationMatrix3 cTargetRotationMatrix(cTargetRotationQuaternion);

               /* copy rotation matrix values */
               cv::Mat(3, 3, CV_64F, &cTargetRotationMatrix(0,0)).copyTo(cTargetTransform(cv::Rect(0,0,3,3)));

               cv::Mat cTargetRelativeTransform = cRootTargetTransform.inv() * cTargetTransform;

               //std::cout << "Target " << it_target->Id << std::endl;
               //std::cout << sTargetBlock << std::endl;
               //print_matrix(std::cout, cTargetRelativeTransform);

               double fBlockWidth = 0.055f;
               std::ostringstream cCoords;

               cCoords << std::round(cTargetRelativeTransform.at<double>(0, 3) / fBlockWidth) << ", ";
               cCoords << std::round(cTargetRelativeTransform.at<double>(1, 3) / fBlockWidth) << ", ";
               cCoords << std::round(cTargetRelativeTransform.at<double>(2, 3) / fBlockWidth);

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
               /*
               std::ostringstream cText;
               cText << '[' << cCoords.str() << ']';
               CFrameAnnotator::Annotate(cAnnotatedImage,
                                         *it_target,
                                         m_pcBlockSensor->GetCameraMatrix(),
                                         m_pcBlockSensor->GetDistortionParameters(),
                                         cText.str(),
                                         cColor);

               */

            }
         }
                  
         cv::Mat cOutputImg;
         cv::flip(cAnnotatedImage, cOutputImg, 1);
         cv::imshow("Output", cAnnotatedImage);

         /*
         std::ostringstream cFilePath;
         cFilePath << "/home/allsey87/Workspace/blocktracker-testbench/output/"
                   << "out"
                   << std::setfill('0')
                   << std::setw(7)
                   << static_cast<int>(fTimestamp)
                   << ".png";
         cv::imwrite(cFilePath.str().c_str(), cAnnotatedImage);
         */

         /* delay */
         if(cv::waitKey(1) == 'q') {
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

