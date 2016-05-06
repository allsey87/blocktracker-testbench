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

#include "block_sensor.h"
#include "block_tracker.h"
#include "structure_analyser.h"
#include "frame_annotator.h"

#include <argos3/core/utility/math/matrix/rotationmatrix3.h>
#include <argos3/core/utility/math/quaternion.h>
#include <argos3/core/utility/math/angles.h>

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
   bool bWebcamMode = (unLoadedImagesCount == 0);
   

   if(bWebcamMode) {
      std::cerr << "Capturing from webcam" << std::endl; 
   }
   else {
      std::cerr << "Loaded " << unLoadedImagesCount << " images" << std::endl; 
      /* ensure images are in the correct order w.r.t. timestamps */
      std::sort(std::begin(vecImages),
                std::end(vecImages),
                [] (const SLoadedImage& s_lhs, const SLoadedImage& s_rhs) {
                  return s_lhs.Timestamp < s_rhs.Timestamp;
                }
      );
   }

   /* Parameters for the camera, no loaded images => creative webcam parameters */
   /* camera focal length in pixels */
   const double m_fFx = bWebcamMode ?  6.8524376287599046e+02 :  8.8396142504070610e+02; 
   const double m_fFy = bWebcamMode ?  6.8524376287599046e+02 :  8.8396142504070610e+02; 
   /* camera principal point */
   const double m_fPx = bWebcamMode ?  3.1950000000000000e+02 :  3.1950000000000000e+02;
   const double m_fPy = bWebcamMode ?  2.3950000000000000e+02 :  1.7950000000000000e+02;
   /* camera distortion coefficients */
   const double m_fK1 = bWebcamMode ?  9.6397225197803263e-02 :  1.8433447851104852e-02;
   const double m_fK2 = bWebcamMode ? -7.5356253832909403e-01 :  1.6727474183089033e-01;
   const double m_fK3 = bWebcamMode ?  1.4040437078320873e+00 : -1.5480889084966631e+00;
   /* camera matrix */
   const cv::Matx<double, 3, 3> cCameraMatrix = 
      cv::Matx<double, 3, 3>(m_fFx, 0.0f, m_fPx,
                             0.0f, m_fFy, m_fPy,
                             0.0f,  0.0f,  1.0f);
   /* camera distortion parameters */
   const cv::Matx<double, 5, 1> cDistortionParameters =
      cv::Matx<double, 5, 1>(m_fK1, m_fK2, 0.0f, 0.0f, m_fK3);

   CBlockSensor* m_pcBlockSensor =
      new CBlockSensor(cCameraMatrix, cDistortionParameters);
   CBlockTracker* m_pcBlockTracker =
      new CBlockTracker(3u, 0.05f);
   CStructureAnalyser* m_pcStructureAnalyser =
      new CStructureAnalyser;

   SBlock::TList tDetectedBlocksList;
   STarget::TList tTrackedTargetList;
   SStructure::TList tStructureList;
      
   cv::namedWindow("Output");
   cv::moveWindow("Output", 1975, 50);
 
   CFrameAnnotator cFrameAnnotator(cCameraMatrix, cDistortionParameters);

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
         /* clear list of blocks from previous detection */                 
         tDetectedBlocksList.clear();

         /* convert image to apriltags format */
         image_u8_t* ptImageY = image_u8_create(s_loaded_image.ImageData.cols, s_loaded_image.ImageData.rows);
         for (unsigned int un_row = 0; un_row < ptImageY->height; un_row++) {
            std::memcpy(&ptImageY->buf[un_row * ptImageY->stride],
            s_loaded_image.ImageData.row(un_row).data,
            ptImageY->width);
         }         
         /* Detect blocks */
         m_pcBlockSensor->DetectBlocks(ptImageY, ptImageY, ptImageY, tDetectedBlocksList);
         /* Deallocate the apriltags image */
         image_u8_destroy(ptImageY);

         /* Associate the known targets with the newly detected blocks */
         m_pcBlockTracker->AssociateAndTrackTargets(s_loaded_image.Timestamp, tDetectedBlocksList, tTrackedTargetList);

         /* Do structure detection */
         m_pcStructureAnalyser->DetectStructures(tTrackedTargetList, tStructureList);

         for(const SStructure& s_structure : tStructureList) {
            cv::Scalar cColor(0.0f,0.0f,0.0f);
            switch(s_structure.Members.size()) {
            case 1:
               cColor = cv::Scalar(0,0,255);
               break;
            case 2:
               cColor = cv::Scalar(0,255,0);
               break;
            case 3:
               cColor = cv::Scalar(255,0,0);
               break;
            default:
               cColor = cv::Scalar(0,255,255);
               break;
            }
            cFrameAnnotator.Annotate(s_structure, cColor);
         }

         /* Create a color version of the image for the annotation */
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
                   << std::chrono::duration<int, std::milli>(s_loaded_image.Timestamp - tReferenceTime).count()
                   << ".png";
         cv::imwrite(cFilePath.str().c_str(), s_loaded_image.ImageData);
*/

         /* delay */
         if(cv::waitKey(200) == 'q') {
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

