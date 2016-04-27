#ifndef TAG_H
#define TAG_H

#include <iostream>
#include <argos3/core/utility/math/matrix/transformationmatrix3.h>
#include <opencv2/core/core.hpp>

#include <argos3/core/utility/math/vector3.h>
#include <argos3/core/utility/math/quaternion.h>

enum class ELedState {
   OFF, Q1, Q2, Q3, Q4,
};

struct STag {
   argos::CTransformationMatrix3 TransformationMatrix;

   std::vector<std::pair<double, double>> Corners;
   std::pair<double, double> Center;
   std::vector<ELedState> DetectedLeds;

   argos::CQuaternion Rotation;
   argos::CVector3 Translation;
  
   cv::Mat RotationVector;
   cv::Mat TranslationVector;
};

std::ostream& operator<<(std::ostream& c_output_stream, ELedState e_led_state);

#endif
