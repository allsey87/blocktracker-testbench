#ifndef BLOCK_H
#define BLOCK_H

#include <chrono>

#include "tag.h"

struct SBlock {
   /* Timestamp marking when the block was detected */
   std::chrono::time_point<std::chrono::steady_clock> Timestamp;
   /* Set of tags used to identify the block */
   std::vector<STag> Tags;

   /* Rotation and translations matrices */
   argos::CQuaternion RotationT;
   argos::CVector3 TranslationT;

   cv::Mat RotationVector, TranslationVector;
   /* Block cartesian coordinates and euler angles */
   struct {
      double X = 0.0f, Y = 0.0f, Z = 0.0f;
   } Translation;
   struct {
      double Z = 0.0f, Y = 0.0f, X = 0.0f;
   } Rotation;
   /* Hack - remove me */
   std::vector<STag> HackTags;
};

#endif
