#ifndef BLOCK_H
#define BLOCK_H

#include <chrono>

#include "tag.h"

struct SBlock {
   SBlock(const std::chrono::time_point<std::chrono::steady_clock>& c_timestamp = 
             std::chrono::time_point<std::chrono::steady_clock>(),
          const argos::CVector3& c_translation =
             argos::CVector3(0.0f, 0.0f, 0.0f),
          const argos::CQuaternion& c_rotation =
             argos::CQuaternion(1.0f, 0.0f, 0.0f, 0.0f)) :
      Timestamp(c_timestamp),
      Translation(c_translation),
      Rotation(c_rotation) {}

   /* Timestamp marking when the block was detected */
   std::chrono::time_point<std::chrono::steady_clock> Timestamp;

   /* Rotation (normalized) and translation */
   argos::CVector3 Translation;   
   argos::CQuaternion Rotation;

   /* 2D coordinates of the block in the frame */
   argos::CVector2 Coordinates;

   /* Set of tags used to identify the block */
   std::vector<STag> Tags;
   /* Hack - remove me */
   std::vector<STag> HackTags;

   /* RotationVector and TranslationVectors from cv::composeRT */
   cv::Matx31d RotationVector;
   cv::Matx31d TranslationVector;
};

#endif
