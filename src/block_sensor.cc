
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

void CBlockSensor::DetectBlocks(const cv::Mat& c_grayscale_frame,
                                std::list<SBlock>& lst_blocks) {
                                
   
                                
   /* Create a list for the detections */
   std::list<SBlock> lst_detections;
   /* extract tags from frame */
   std::vector<AprilTags::TagDetection> vecDetections =
      m_cTagDetector.extractTags(c_grayscale_frame);
      
      
   std::vector<argos::CQuaternion> vecResults; 

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

      //std::cerr << "Translation: " << sBlock.TranslationVector << std::endl;
      
      cv::Matx33f cThisRotationMatrixCV, cReferenceRotationMatrixCV;

      cv::Rodrigues(sBlock.RotationVector, cThisRotationMatrixCV);
      //cv::Rodrigues(lst_detections.front().RotationVector, cReferenceRotationMatrixCV);
      
      argos::CRotationMatrix3 cThisRotationMatrix;
      //argos::CRotationMatrix3 cReferenceRotationMatrix;
      
      cThisRotationMatrix.Set(&cThisRotationMatrixCV(0,0));
      //cReferenceRotationMatrix.Set(&cReferenceRotationMatrixCV(0,0));
      
      argos::CQuaternion cThisRotationQuaternion = cThisRotationMatrix.ToQuaternion();
      argos::CQuaternion cReferenceRotationQuaternion(argos::CRadians::ZERO, argos::CVector3::Z);
      
      //std::cerr << "This block rotation: " << cThisRotationQuaternion << std::endl;
      //std::cerr << "Reference block rotation: " << cReferenceRotationQuaternion << std::endl;
      
      argos::CQuaternion cTransfer(cReferenceRotationQuaternion * cThisRotationQuaternion.Inverse());
      
      //std::cerr << "Transfer rotation: " << cTransfer << std::endl;

      argos::CRadians pcEulerAngles[3];
      argos::CQuaternion cResult;
      
      cTransfer.ToEulerAngles(pcEulerAngles[0], pcEulerAngles[1], pcEulerAngles[2]);

      for(argos::CRadians& cEulerAngle : pcEulerAngles) {
         cEulerAngle.SetValue(std::round(cEulerAngle.GetValue() / (0.5f * ARGOS_PI)) * (0.5f * ARGOS_PI));
      }         
      
      cTransfer.FromEulerAngles(pcEulerAngles[0], argos::CRadians::ZERO, argos::CRadians::ZERO);
      cResult = cTransfer * cThisRotationQuaternion;
      
      cTransfer.FromEulerAngles(argos::CRadians::ZERO, pcEulerAngles[1], argos::CRadians::ZERO);
      cResult = cTransfer * cResult;

      cTransfer.FromEulerAngles(argos::CRadians::ZERO, argos::CRadians::ZERO, pcEulerAngles[2]);
      cResult = cTransfer * cResult;
      
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

void CBlockSensor::ToStandardRepresentation(const cv::Matx31f& c_rotation_vector,
                                            const cv::Matx31f& c_translation_vector,
                                            float& f_x, float& f_y, float& f_z,
                                            float& f_yaw, float& f_pitch, float& f_roll) {

   /* Extract the position and rotation of the block, relative to the camera */
   cv::Matx33f cR;
   cv::Rodrigues(c_rotation_vector, cR);
   cv::Matx44f cT(cR(0,0), cR(0,1), cR(0,2), c_translation_vector(0),
                  cR(1,0), cR(1,1), cR(1,2), c_translation_vector(1),
                  cR(2,0), cR(2,1), cR(2,2), c_translation_vector(2),
                  0,       0,       0,       1);
   cv::Matx44f cM( 0,  0,  1,  0,
                  -1,  0,  0,  0,
                   0, -1,  0,  0,
                   0,  0,  0,  1);
   cv::Matx44f cMT(cM, cT, cv::Matx_MatMulOp());

   cv::Matx33f cF( 1,  0,  0, 
                   0, -1,  0,
                   0,  0,  1);
   cv::Matx33f cFR(cF, cR, cv::Matx_MatMulOp());

   /* Store the 3D coordinates of the block into the struct */
   f_x = cMT(0,3);
   f_y = cMT(1,3);
   f_z = cMT(2,3);
   f_yaw = StandardRad(atan2(cFR(1,0), cFR(0,0)));
   f_pitch = StandardRad(atan2(-cFR(2,0), cFR(0,0) * cos(f_yaw) + cFR(1,0) * sin(f_yaw)));
   f_roll  = StandardRad(atan2( cFR(0,2) * sin(f_yaw) - cFR(1,2) * cos(f_yaw),
                               -cFR(0,1) * sin(f_yaw) + cFR(1,1) * cos(f_yaw)));
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

