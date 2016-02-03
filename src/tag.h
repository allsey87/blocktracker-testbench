#ifndef TAG_H
#define TAG_H

#include <opencv2/core/core.hpp>

enum class ELedState {
   OFF, Q1, Q2, Q3, Q4,
};

struct STag {
   std::vector<std::pair<float, float>> Corners;
   std::pair<float, float> Center;
   cv::Mat RotationVector;
   cv::Mat TranslationVector;
   
   std::vector<ELedState> LedStates;  
};

#endif
