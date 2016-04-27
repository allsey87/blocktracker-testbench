#ifndef BLOCK_SENSOR_H
#define BLOCK_SENSOR_H

#include <opencv2/core/core.hpp>
#include <apriltag/image_u8.h>
#include <argos3/core/utility/math/vector3.h>

#include "block.h"

#include <list>

struct apriltag_family;
struct apriltag_detector;

class CBlockSensor {
   
public:

   /* constructor */
   CBlockSensor();
   
   /* destructor */
   ~CBlockSensor();

   void DetectBlocks(image_u8_t* pt_y_frame,
                     image_u8_t* pt_u_frame,
                     image_u8_t* pt_v_frame,
                     std::list<SBlock>& lst_blocks);

   const cv::Matx33d& GetCameraMatrix() {
      return m_cCameraMatrix;
   }

   const cv::Vec4d& GetDistortionParameters() {
      return m_cDistortionParameters;
   }

private:

   void DetectLeds(STag& s_tag, image_u8_t* pt_y_frame, image_u8_t* pt_u_frame, image_u8_t* pt_v_frame);

   void ClusterDetections(std::list<SBlock>& lst_detections,
                          std::list<SBlock>& lst_blocks);
                          
   /* Apriltag family and detector */
   apriltag_family* m_psTagFamily;
   apriltag_detector* m_psTagDetector;
   
   /* Apriltag (w.r.t. black frame) and block side length in meters */
   const double m_fTagSize = 0.024;
   const double m_fBlockSideLength = 0.055;
   const double m_fInterLedLength = 0.040;
   const unsigned int m_unLedRegionOfInterestLength = 9;
   const unsigned int m_unLedLuminanceOnThreshold = 64;
   
   /* camera focal length in pixels */
   const double m_fFx = 555.0; 
   const double m_fFy = 555.0;
   
   /* camera principal point */
   const double m_fPx = 320.0; 
   const double m_fPy = 180.0;

   /* Tag to block translation and rotation constants */
   const cv::Matx31d m_cTagToBlockTranslationCV = cv::Matx31d(0, 0, m_fBlockSideLength / 2);
   const cv::Matx31d m_cTagToBlockRotationCV = cv::Matx31d(0, 0, 0);

   const argos::CVector3 m_cTagToBlockTranslation = argos::CVector3(0, 0, -m_fBlockSideLength / 2);

   const argos::CTransformationMatrix3 m_cCameraToModelTransform =
      argos::CTransformationMatrix3(-1.0f,  0.0f,  0.0f,  0.0f,
                                     0.0f, -1.0f,  0.0f,  0.0f,
                                     0.0f,  0.0f,  1.0f,  0.0f,
                                     0.0f,  0.0f,  0.0f,  1.0f);

   /* corner locations of the april tag */
   const std::vector<cv::Point3d> m_vecTagPts = {
      cv::Point3d(-m_fTagSize/2., -m_fTagSize/2., 0),
      cv::Point3d( m_fTagSize/2., -m_fTagSize/2., 0),
      cv::Point3d( m_fTagSize/2.,  m_fTagSize/2., 0),
      cv::Point3d(-m_fTagSize/2.,  m_fTagSize/2., 0)
   };

   /* camera matrix */
   const cv::Matx33d m_cCameraMatrix = cv::Matx33d(m_fFx, 0, m_fPx,
                                                   0, m_fFy, m_fPy,
                                                   0,     0,    1);
   /* camera distortion parameters */
   const cv::Vec4d m_cDistortionParameters = cv::Vec4d(0, 0, 0, 0);

   
};

#endif


