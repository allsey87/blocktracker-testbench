#ifndef BLOCK_H
#define BLOCK_H

#include <chrono>

#include "tag.h"

struct SBlock {
   /* Timestamp marking when the block was detected */
   std::chrono::time_point<std::chrono::steady_clock> Timestamp;
   /* Set of tags used to identify the block */
   std::vector<STag> Tags;
   /* Block 2D coordinates in frame */
   std::pair<float, float> Coordinates;
   /* Rotation and translations matrices */
   cv::Mat RotationVector, TranslationVector;
   /* Block cartesian coordinates and euler angles */
   struct {
      float X = 0.0f, Y = 0.0f, Z = 0.0f;
   } Translation;
   struct {
      float Z = 0.0f, Y = 0.0f, X = 0.0f;
   } Rotation;
   /* Hack - remove me */
   std::vector<STag> HackTags;
   /* Flag whether block is real or simulated */
   bool IsPseudo = false;
};

#endif
