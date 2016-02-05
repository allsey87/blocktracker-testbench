
#include "block_sensor.h"

#include "tag.h"
#include "block.h"
#include <argos3/core/utility/math/matrix/rotationmatrix3.h>
#include <argos3/core/utility/math/quaternion.h>
#include <argos3/core/utility/math/angles.h>
#include <argos3/core/utility/math/vector3.h>

/****************************************/
/****************************************/

float StandardRad(double t) {
   if (t >= 0.) {
      t = fmod(t+M_PI, 2*M_PI) - M_PI;
   } else {
      t = fmod(t-M_PI, -2*M_PI) + M_PI;
   }
   return t;
}

/****************************************/
/****************************************/

CBlockSensor::CBlockSensor() {}

/****************************************/
/****************************************/

void CBlockSensor::DetectBlocks(const cv::Mat& c_y_frame,
                                cv::Mat& c_u_frame,
                                cv::Mat& c_v_frame,
                                std::list<SBlock>& lst_blocks) {
                                
   
                                
   /* Create a list for the detections */
   std::list<SBlock> lst_detections;
   /* extract tags from frame */
   std::vector<AprilTags::TagDetection> vecDetections =
      m_cTagDetector.extractTags(c_y_frame);
      
      
   std::vector<argos::CQuaternion> vecResults; 
   
   unsigned int counter_to_remove = 0;

   for(const AprilTags::TagDetection& c_detection : vecDetections) {
      /* create a new block for this detection */
      SBlock sBlock;                                
      /* Add an empty tag to the block and get a reference to it */
      sBlock.Tags.emplace_back();
      STag& sTag = sBlock.Tags.back();
      /* Copy the corners of the tags into an STag for future use */
      sTag.Corners.assign(c_detection.p, c_detection.p + 4);
      sTag.Center = c_detection.cxy;
      /* Create a vector of OpenCV 2D points representing the tag */
      std::vector<cv::Point2f> vecImagePts = {
         cv::Point2f(c_detection.p[0].first, c_detection.p[0].second),
         cv::Point2f(c_detection.p[1].first, c_detection.p[1].second),
         cv::Point2f(c_detection.p[2].first, c_detection.p[2].second),
         cv::Point2f(c_detection.p[3].first, c_detection.p[3].second)
      };
      /* OpenCV SolvePnP - detect the translation between the camera 
         plane and the tag plane */
      cv::solvePnP(m_vecTagPts,
                   vecImagePts,
                   m_cCameraMatrix,
                   m_cDistortionParameters,
                   sTag.RotationVector,
                   sTag.TranslationVector);
                   
      /* Debug - draw axes on each tag */
      std::vector<cv::Point3f> vecAxesPoints = {
         cv::Point3f(0, 0, 0),
         cv::Point3f(m_fTagSize, 0, 0),
         cv::Point3f(0, m_fTagSize, 0),
         cv::Point3f(0, 0, m_fTagSize),
      };

      std::vector<cv::Point2f> vecLedAxesPixels;
      cv::projectPoints(vecAxesPoints,
                        sTag.RotationVector,
                        sTag.TranslationVector,
                        m_cCameraMatrix,
                        m_cDistortionParameters,
                        vecLedAxesPixels);
                              
      cv::line(c_u_frame, vecLedAxesPixels[0], vecLedAxesPixels[1], cv::Scalar(0,0,255), 2);
      cv::line(c_u_frame, vecLedAxesPixels[0], vecLedAxesPixels[2], cv::Scalar(0,255,0), 2);
      cv::line(c_u_frame, vecLedAxesPixels[0], vecLedAxesPixels[3], cv::Scalar(255,0,0), 2);
      
      cv::putText(c_u_frame, std::to_string(counter_to_remove++), vecLedAxesPixels[3], cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(255,255,255), 2);

      /* Detect the LEDs */
      std::vector<cv::Point3f> vecLedPoints = {
         cv::Point3f(-m_fInterLedLength * 0.5f, 0, 0),
         cv::Point3f( m_fInterLedLength * 0.5f, 0, 0),
         cv::Point3f( 0, -m_fInterLedLength * 0.5f, 0),
         cv::Point3f( 0,  m_fInterLedLength * 0.5f, 0)
      };
      std::vector<cv::Point2f> vecLedCentrePixels;
      cv::projectPoints(vecLedPoints,
                        sTag.RotationVector,
                        sTag.TranslationVector,
                        m_cCameraMatrix,
                        m_cDistortionParameters,
                        vecLedCentrePixels);
                       
      for(cv::Point2f& c_led_point : vecLedCentrePixels) {
         /* skip this LED if the coordinates are out of range */
         if(c_led_point.x < (m_unLedRegionOfInterestLength / 2u) ||
            c_led_point.x > (c_u_frame.cols - 1) - (m_unLedRegionOfInterestLength / 2u)) {
            continue;   
         }

         if(c_led_point.y < (m_unLedRegionOfInterestLength / 2u) ||
            c_led_point.y > (c_u_frame.rows - 1) - (m_unLedRegionOfInterestLength / 2u)) {
            continue;   
         }
         
         cv::Rect cLedRectangle(c_led_point.x - (m_unLedRegionOfInterestLength / 2u),
                                c_led_point.y - (m_unLedRegionOfInterestLength / 2u),
                                m_unLedRegionOfInterestLength,
                                m_unLedRegionOfInterestLength);
         
         // DEBUG
         cv::rectangle(c_u_frame, cLedRectangle, cv::Scalar(0, 0, 0));
         // DEBUG

         struct {
            cv::Mat Y, U, V;
         } sRegionOfInterest = {
            c_u_frame(cLedRectangle),
            c_u_frame(cLedRectangle),
            c_u_frame(cLedRectangle),
         };
         
         float fSumY = 0.0f, fWeightedAverageU = 0.0f, fWeightedAverageV = 0.0f;
         
         for(unsigned int un_row = 0; un_row < sRegionOfInterest.Y.rows; un_row++) {
            for(unsigned int un_col = 0; un_col < sRegionOfInterest.Y.cols; un_col++) {
               float fPixelY = sRegionOfInterest.Y.at<float>(un_row, un_col);
               fSumY += fPixelY;
               fWeightedAverageU += fPixelY * sRegionOfInterest.U.at<float>(un_row, un_col);
               fWeightedAverageV += fPixelY * sRegionOfInterest.V.at<float>(un_row, un_col);
            }
         }
         
         fWeightedAverageU /= fSumY;
         fWeightedAverageV /= fSumY;
         
         if(fSumY / (m_unLedRegionOfInterestLength * m_unLedRegionOfInterestLength) > 
            m_unLedLuminanceOnThreshold) {
            if(fWeightedAverageU > 128.0f) {
               if(fWeightedAverageV > 128.0f) {
                  sTag.LedStates.push_back(ELedState::Q1);
               }
               else {
                  sTag.LedStates.push_back(ELedState::Q4);
               }
            }
            else {
               if(fWeightedAverageV > 128.0f) {
                  sTag.LedStates.push_back(ELedState::Q2);
               }
               else {
                  sTag.LedStates.push_back(ELedState::Q3);
               }
            }
         }
         else {
            sTag.LedStates.push_back(ELedState::OFF);
         }
      }


      /* Compose the tag-to-block and camera-to-tag transformations to get
         the camera-to-block transformation, storing the result directly 
         inside sBlock */
      cv::composeRT(m_cTagToBlockRotation,
                    m_cTagToBlockTranslation,
                    sTag.RotationVector,
                    sTag.TranslationVector,
                    sBlock.RotationVector,
                    sBlock.TranslationVector);
                    
      /////// DEBUG
      
      /* if there is at least one block already in the vector */
      cv::Matx33f cThisRotationMatrixCV;
      cv::Rodrigues(sBlock.RotationVector, cThisRotationMatrixCV);
      argos::CRotationMatrix3 cThisRotationMatrix;
      cThisRotationMatrix.Set(&cThisRotationMatrixCV(0,0));
      argos::CQuaternion cThisRotationQuaternion = cThisRotationMatrix.ToQuaternion();         

      std::cerr << "cThisRotationQuaternion = " << cThisRotationQuaternion << "(" << cThisRotationQuaternion.Length() << ")" << std::endl;
      
      argos::CRadians cBlockEulerAngles[3];
      
      cThisRotationQuaternion.ToEulerAngles(cBlockEulerAngles[0], cBlockEulerAngles[1], cBlockEulerAngles[2]);
      
      argos::CRange<argos::CRadians> cBlockRotationRange(-argos::CRadians::PI_OVER_FOUR, argos::CRadians::PI_OVER_FOUR);
      
      cBlockRotationRange.WrapValue(cBlockEulerAngles[0]);
      
      std::cerr << "cThisRotationQuaternion[0] = " << cBlockEulerAngles[0] << std::endl;
      
      
      
      if(lst_detections.size() > 0) {

         std::cerr << "-------------- " << counter_to_remove - 1 << " --------------" << std::endl;
         cv::Matx33f cReferenceRotationMatrixCV;
         cv::Rodrigues(lst_detections.front().RotationVector, cReferenceRotationMatrixCV);
         argos::CRotationMatrix3 cReferenceRotationMatrix;
         cReferenceRotationMatrix.Set(&cReferenceRotationMatrixCV(0,0));
         argos::CQuaternion cReferenceRotationQuaternion = cReferenceRotationMatrix.ToQuaternion();
         
         std::cerr << "cReferenceRotationQuaternion = " << cReferenceRotationQuaternion << "(" << cReferenceRotationQuaternion.Length() << ")" << std::endl;

         
         //argos::CQuaternion cReferenceRotationQuaternion(argos::CRadians::ZERO, argos::CVector3::Z);
         
         //std::cerr << "This block rotation: " << cThisRotationQuaternion << std::endl;
         //std::cerr << "Reference block rotation: " << cReferenceRotationQuaternion << std::endl;
         
         argos::CQuaternion cTransfer(cReferenceRotationQuaternion * cThisRotationQuaternion.Inverse());
         
         std::cerr << "cTransfer = " << cTransfer << std::endl;

         argos::CRadians pcEulerAngles[3];
         argos::CQuaternion cResult;
         
         cTransfer.ToEulerAngles(pcEulerAngles[0], pcEulerAngles[1], pcEulerAngles[2]);

         for(argos::CRadians& cEulerAngle : pcEulerAngles) {
            cEulerAngle.UnsignedNormalize();
            //cEulerAngle.SetValue(std::round(cEulerAngle.GetValue() / (0.5f * ARGOS_PI)) * (0.5f * ARGOS_PI));
         }         
         
         enum class EAxis {
            Z = 0, Y = 1, X = 2
         };
         
         auto EAxisToString = [] (EAxis e_axis) {
            switch(e_axis) {
               case EAxis::Z:
                  return std::string("Z");
                  break;
               case EAxis::Y:
                  return std::string("Y");
                  break;
               case EAxis::X:
                  return std::string("X");
                  break;
            }
         };
                          
         std::map<EAxis, std::pair<EAxis, bool>> mapTranslations = {
            { EAxis::Z, {EAxis::Z, false} },
            { EAxis::Y, {EAxis::Y, false} },
            { EAxis::X, {EAxis::X, false} }, 
         };
         
         std::vector<std::pair<EAxis, argos::UInt8>> vecReqRotations = {
            { EAxis::Z, std::round(pcEulerAngles[0].GetValue() / argos::CRadians::PI_OVER_TWO.GetValue()) },
            { EAxis::Y, std::round(pcEulerAngles[1].GetValue() / argos::CRadians::PI_OVER_TWO.GetValue()) },
            { EAxis::X, std::round(pcEulerAngles[2].GetValue() / argos::CRadians::PI_OVER_TWO.GetValue()) },
         };
         
         std::set<std::pair<EAxis, EAxis>> setSignChangeReq = {
            { EAxis::X, EAxis::Y },
            { EAxis::Y, EAxis::Z },
            { EAxis::Z, EAxis::X },
         };
                  
         /* Consider the rotation on each axis */        
         for(const std::pair<EAxis, argos::UInt8>& t_rotation_steps : vecReqRotations) {
            /* For each translation */
            for(std::pair<const EAxis, std::pair<EAxis, bool>>& t_translation : mapTranslations) {
               /* if the rotation occurs along the axis of the current translation, skip */
               if(t_translation.second.first == t_rotation_steps.first) {
                  continue;
               }
               else {
                  /* for each rotation step */
                  for(argos::UInt8 un_rotation_steps = 0; 
                      un_rotation_steps < t_rotation_steps.second;
                      un_rotation_steps++) {
                     /* Find the axis e_axis to swap to */
                     for(EAxis e_axis : {EAxis::Z, EAxis::Y, EAxis::X}) {
                        if(e_axis != t_translation.second.first &&
                           e_axis != t_rotation_steps.first) {
                           if(setSignChangeReq.count(std::make_pair(t_translation.second.first, e_axis)) == 1) {
                              /* invert the axis */
                              t_translation.second.second = !t_translation.second.second;
                           }
                           t_translation.second.first = e_axis;
                           break;
                        }
                     }
                  }
               }
            }
         }
         
         cThisRotationQuaternion.ToEulerAngles(pcEulerAngles[0], pcEulerAngles[1], pcEulerAngles[2]);
         
         std::cerr << "Before mapping: ";
         for(EAxis e_axis : {EAxis::Z, EAxis::Y, EAxis::X}) {
            std::cerr << EAxisToString(e_axis) << ":" << pcEulerAngles[static_cast<int>(e_axis)].GetValue() << ", ";
         }
         std::cerr << std::endl;
                 
         for(std::pair<const EAxis, std::pair<EAxis, bool>> t_translation : mapTranslations) {
            std::cerr << EAxisToString(t_translation.first) << " -> " << (t_translation.second.second ? "-" : "") << EAxisToString(t_translation.second.first) << std::endl;
         }
         
         cResult.FromEulerAngles(pcEulerAngles[static_cast<int>(mapTranslations[EAxis::Z].first)] * (mapTranslations[EAxis::Z].second ? -1 : 1),
                                 pcEulerAngles[static_cast<int>(mapTranslations[EAxis::Y].first)] * (mapTranslations[EAxis::Y].second ? -1 : 1),
                                 pcEulerAngles[static_cast<int>(mapTranslations[EAxis::X].first)] * (mapTranslations[EAxis::X].second ? -1 : 1));
                                 
         cResult.ToEulerAngles(pcEulerAngles[0], pcEulerAngles[1], pcEulerAngles[2]);
         
                
         std::cerr << "After mapping: ";
         for(EAxis e_axis : {EAxis::Z, EAxis::Y, EAxis::X}) {
            std::cerr << EAxisToString(e_axis) << ":" << pcEulerAngles[static_cast<int>(e_axis)].GetValue() << ", ";
         }
         std::cerr << std::endl;
         
         //std::cerr << "Rotation[" << lst_detections.size() << "]: " << cResult << std::endl;
         vecResults.push_back(cResult);
               
         ////// ZE BUGS
                       
         
         /* Extract the position and rotation of the block, relative to the camera
         ToStandardRepresentation(sBlock.RotationVector, 
                                  sBlock.TranslationVector,
                                  sBlock.X,
                                  sBlock.Y,
                                  sBlock.Z,
                                  sBlock.Yaw,
                                  sBlock.Pitch,
                                  sBlock.Roll); 
         */

         /* compute the 2D coordinates of the block */
         
         /*
         std::vector<cv::Point3f> vecCentrePoint = {
            cv::Point3f(0,0,0)
         };
         std::vector<cv::Point2f> vecCentrePixel;
         cv::projectPoints(vecCentrePoint,
                           sBlock.RotationVector,
                           sBlock.TranslationVector,
                           m_cCameraMatrix,
                           m_cDistortionParameters,
                           vecCentrePixel);
         sBlock.Coordinates = std::pair<float, float>(vecCentrePixel[0].x, vecCentrePixel[0].y);
         */
      }
      /* store the block into our block list */
      lst_detections.push_back(sBlock);
   }
   /* cluster the blocks */
   //ClusterDetections(lst_detections, lst_blocks);
   argos::CQuaternion cEndResult;
   
   if(vecResults.size() > 0 ) {
      argos::CRadians pcEulerAngles[3];   
      for(const argos::CQuaternion& c_result : vecResults) {
         std::cerr << c_result << std::endl;
         argos::CRadians pcEulerAnglesTemp[3];
         c_result.ToEulerAngles(pcEulerAnglesTemp[0], pcEulerAnglesTemp[1], pcEulerAnglesTemp[2]);
         pcEulerAngles[0] += pcEulerAnglesTemp[0];
         pcEulerAngles[1] += pcEulerAnglesTemp[1];
         pcEulerAngles[2] += pcEulerAnglesTemp[2];
      }
      cEndResult.FromEulerAngles(pcEulerAngles[0] / static_cast<float>(vecResults.size()),
                                 pcEulerAngles[1] / static_cast<float>(vecResults.size()),
                                 pcEulerAngles[2] / static_cast<float>(vecResults.size()));
      std::cerr << "average:" << std::endl << cEndResult << std::endl << std::endl;
   }

   
   
   
   lst_detections.swap(lst_blocks);
}


/****************************************/
/****************************************/

void CBlockSensor::ClusterDetections(std::list<SBlock>& lst_detections,
                                     std::list<SBlock>& lst_blocks) {
   /* a working list of clusters */
   TClusterList lstClusters;
   /* loop until we have allocated all of our detections into clusters */
   while(!lst_detections.empty()) {
      /* take a reference to the first block in the detections list */
      std::list<SBlock>::iterator itDetectedBlock = std::begin(lst_detections);
      /* keep a list of interators into the matching clusters */
      std::list<TClusterList::iterator> lstBlockToClusterAssignments;
      /* for each cluster */
      for(TClusterList::iterator it_cluster = std::begin(lstClusters);
          it_cluster != std::end(lstClusters);
          it_cluster++) {
         /* for each block in the cluster */
         for(TCluster::iterator it_block = it_cluster->begin();
             it_block != it_cluster->end();
             it_block++) {
            float fInterblockDist = sqrt(pow(it_block->X - itDetectedBlock->X, 2) +
                                         pow(it_block->Y - itDetectedBlock->Y, 2) +
                                         pow(it_block->Z - itDetectedBlock->Z, 2));
            /* if the given block in this cluster is within a distance of 
               (m_fBlockSideLength / 2) of the detected block, they belong 
               to the same cluster */
            if(fInterblockDist < (m_fBlockSideLength / 2)) {
               lstBlockToClusterAssignments.push_back(it_cluster);
               /* at this point we know that this cluster is a 
                  candidate for the block and we can stop */
               break;
            }
         }
      }
      /* At this point we have searched all the clusters */
      if(lstBlockToClusterAssignments.size() == 0) {
         /* no matches - create a new cluster in the cluster list */
         lstClusters.emplace_back();
         /* take a reference to the newly created cluster */
         TCluster& tCluster = lstClusters.back();
         /* move our detected block into the cluster */
         tCluster.splice(std::begin(tCluster),
                         lst_detections,
                         itDetectedBlock);
      }
      else {        
         /* move the detected block into the first matching clusters */
         TClusterList::iterator itCluster = lstBlockToClusterAssignments.front();
         /* add the detected block into the first matching cluster */
         itCluster->splice(std::begin(*itCluster),
                           lst_detections,
                           itDetectedBlock);
         /* if there was more than one matching cluster, merge them */
         if(lstBlockToClusterAssignments.size() > 1) {
            /* take an iterator to the first (destination) cluster */
            std::list<TClusterList::iterator>::iterator itDestinationCluster =
               std::begin(lstBlockToClusterAssignments);
            /* move the blocks from the other (source) clusters into the destination cluster */
            for(std::list<TClusterList::iterator>::iterator itSourceCluster = std::next(itDestinationCluster);
                itSourceCluster != std::end(lstBlockToClusterAssignments);
                itSourceCluster++) {
               /* move all the blocks from the source cluster to the destination cluster */
               (*itDestinationCluster)->splice(std::begin(**itDestinationCluster), **itSourceCluster);
               /* remove the empty source cluster */
               lstClusters.erase(*itSourceCluster);
            }
         }
      }
   }

   /////////// DEBUG

   unsigned int i = 0;
   for(TCluster& t_cluster : lstClusters) {
      std::cerr << "cluster " << i << " contains:" << std::endl;
      unsigned int j = 0;
      for(SBlock& s_block : t_cluster) {
         std::cerr << "block " << j << std::endl
                   << "Translation Vector: " << std::endl
                   << s_block.TranslationVector << std::endl
                   << "Rotation Vector: " << std::endl
                   << s_block.RotationVector << std::endl
                   << "Yaw, Pitch, Roll: " << std::endl
                   << s_block.Yaw << ", " << s_block.Pitch << ", " << s_block.Roll << std::endl;

         j++;
      }
      i++;
   }


   /////////// DEBUG

   
   /* find the average location of the block */  
   for(TCluster& t_cluster : lstClusters) {
      float fX = 0.0f, fY = 0.0f, fZ = 0.0f;
      for(SBlock& s_block : t_cluster) {
         fX += s_block.X;
         fY += s_block.Y;
         fZ += s_block.Z;
      }
      fX /= t_cluster.size();
      fY /= t_cluster.size();
      fZ /= t_cluster.size();
      lst_blocks.emplace_back(fX, fY, fZ, 0.0, 0.0, 0.0);
   }
}

/****************************************/
/****************************************/

