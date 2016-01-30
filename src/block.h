#ifndef BLOCK_H
#define BLOCK_H

#include "tag.h"

struct SBlock {
   /* default constructor */
   SBlock() :
      X(0.0f), Y(0.0f), Z(0.0f), Yaw(0.0f), Pitch(0.0f), Roll(0.0f) {}
   /* constructor */
   SBlock(float f_x, float f_y, float f_z, float f_yaw, float f_pitch, float f_roll) :
      X(f_x), Y(f_y), Z(f_z), Yaw(f_yaw), Pitch(f_pitch), Roll(f_roll) {}
   /* Set of tags used to identify the block */
   std::vector<STag> Tags;
   /* Block 2D coordinates in frame */
   std::pair<float, float> Coordinates;
   /* Rotation and translations matrices */
   cv::Mat RotationMatrix, TranslationMatrix;
   /* Block 3D coordinates and orientation */
   float X, Y, Z, Yaw, Pitch, Roll;
   
   
   cv::Mat RotationMatrixTest, TranslationMatrixTest;
};

#endif
