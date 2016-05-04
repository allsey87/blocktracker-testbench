#ifndef FRAME_ANNOTATOR_H
#define FRAME_ANNOTATOR_H

#include <opencv2/core/core.hpp>
#include <functional>

struct STag;
struct SBlock;
struct STarget;

class CFrameAnnotator {
public:
   CFrameAnnotator(const cv::Matx33f& c_camera_matrix,
                   const cv::Vec4f& c_distortion_parameters) :
      m_cCameraMatrix(c_camera_matrix),
      m_cDistortionParameters(c_distortion_parameters) {}

   void Annotate(const STag& s_tag,
                 const cv::Scalar& c_color,
                 int n_thickness = 1,
                 const std::string& str_label = "");

   void Annotate(const SBlock& s_block,
                 const cv::Scalar& c_color,
                 const std::string& str_text = "");

   void Annotate(const STarget& s_target,
                 const cv::Scalar& c_color,
                 const std::string& str_text = "");

   void SetLineThickness(int n_line_thickness) {
      m_nLineThickness = n_line_thickness;
   }

   void Label(const cv::Point2f& c_origin,
              const std::string& str_text);

   void WriteToFrame(cv::Mat& c_frame);

   void Clear();

private:
   cv::Matx33f m_cCameraMatrix;
   cv::Vec4f m_cDistortionParameters;

   int m_nLineThickness = 1;

   std::vector<std::function<void(cv::Mat&)>> m_vecLabelBackgrounds;
   std::vector<std::function<void(cv::Mat&)>> m_vecLabels;
   std::vector<std::function<void(cv::Mat&)>> m_vecLines;

   const std::vector<cv::Point3f> m_vecBlockVertices = {     
      cv::Point3f( 0.0275,  0.0275, 0.0275),
      cv::Point3f( 0.0275, -0.0275, 0.0275),
      cv::Point3f(-0.0275, -0.0275, 0.0275),
      cv::Point3f(-0.0275,  0.0275, 0.0275),
      cv::Point3f( 0.0275,  0.0275, -0.0275),
      cv::Point3f( 0.0275, -0.0275, -0.0275),
      cv::Point3f(-0.0275, -0.0275, -0.0275),
      cv::Point3f(-0.0275,  0.0275, -0.0275)
   };
};


#endif
