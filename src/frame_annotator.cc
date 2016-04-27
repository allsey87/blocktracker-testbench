
#include "frame_annotator.h"

#include "tag.h"
#include "block.h"
#include "target.h"

/* for cv::projectPoints */
#include <opencv2/calib3d/calib3d.hpp>

#include <argos3/core/utility/math/matrix/rotationmatrix3.h>

/****************************************/
/****************************************/

void CFrameAnnotator::Annotate(cv::Mat& c_frame,
                               const SBlock& s_block,
                               const cv::Matx33f& c_camera_matrix,
                               const cv::Vec4f& c_distortion_parameters,
                               const std::string& s_text,
                               const cv::Scalar& c_color,
                               bool b_draw_thick) {
   /* project points is checking correctness */
   std::vector<cv::Point3f> vecInputTargetPoints;
   std::vector<cv::Point2f> vecOutputImagePoints;
      
   vecInputTargetPoints.push_back(cv::Point3f( 0.0275,  0.0275, 0.0275));
   vecInputTargetPoints.push_back(cv::Point3f( 0.0275, -0.0275, 0.0275));
   vecInputTargetPoints.push_back(cv::Point3f(-0.0275, -0.0275, 0.0275));
   vecInputTargetPoints.push_back(cv::Point3f(-0.0275,  0.0275, 0.0275));
   vecInputTargetPoints.push_back(cv::Point3f( 0.0275,  0.0275, -0.0275));
   vecInputTargetPoints.push_back(cv::Point3f( 0.0275, -0.0275, -0.0275));
   vecInputTargetPoints.push_back(cv::Point3f(-0.0275, -0.0275, -0.0275));
   vecInputTargetPoints.push_back(cv::Point3f(-0.0275,  0.0275, -0.0275));

   cv::projectPoints(vecInputTargetPoints,
                     s_block.RotationVector,
                     s_block.TranslationVector,
                     c_camera_matrix,
                     c_distortion_parameters,
                     vecOutputImagePoints);

   cv::line(c_frame, vecOutputImagePoints[0], vecOutputImagePoints[1], c_color, b_draw_thick ? 2 : 1);
   cv::line(c_frame, vecOutputImagePoints[1], vecOutputImagePoints[2], c_color, b_draw_thick ? 2 : 1);
   cv::line(c_frame, vecOutputImagePoints[2], vecOutputImagePoints[3], c_color, b_draw_thick ? 2 : 1);
   cv::line(c_frame, vecOutputImagePoints[3], vecOutputImagePoints[0], c_color, b_draw_thick ? 2 : 1);

   cv::line(c_frame, vecOutputImagePoints[4], vecOutputImagePoints[5], c_color, b_draw_thick ? 2 : 1);
   cv::line(c_frame, vecOutputImagePoints[5], vecOutputImagePoints[6], c_color, b_draw_thick ? 2 : 1);
   cv::line(c_frame, vecOutputImagePoints[6], vecOutputImagePoints[7], c_color, b_draw_thick ? 2 : 1);
   cv::line(c_frame, vecOutputImagePoints[7], vecOutputImagePoints[4], c_color, b_draw_thick ? 2 : 1);

   cv::line(c_frame, vecOutputImagePoints[0], vecOutputImagePoints[4], c_color, b_draw_thick ? 2 : 1);
   cv::line(c_frame, vecOutputImagePoints[1], vecOutputImagePoints[5], c_color, b_draw_thick ? 2 : 1);
   cv::line(c_frame, vecOutputImagePoints[2], vecOutputImagePoints[6], c_color, b_draw_thick ? 2 : 1);
   cv::line(c_frame, vecOutputImagePoints[3], vecOutputImagePoints[7], c_color, b_draw_thick ? 2 : 1);

   for(const STag& s_tag : s_block.Tags) {
      Annotate(c_frame, s_tag);
   }
   for(const STag& s_tag : s_block.HackTags) {
      Annotate(c_frame, s_tag);
   }
}

/****************************************/
/****************************************/

void CFrameAnnotator::Annotate(cv::Mat& c_frame, const STag& s_tag, const std::string& s_text) {
   for(uint8_t un_corner_idx = 0; un_corner_idx < 4; un_corner_idx++) {
      cv::line(c_frame,
               cv::Point2f(s_tag.Corners[un_corner_idx].first, s_tag.Corners[un_corner_idx].second),
               cv::Point2f(s_tag.Corners[(un_corner_idx + 1) % 4].first, s_tag.Corners[(un_corner_idx + 1) % 4].second),
               cv::Scalar(0,0,0),
               1);
   }
   if(!s_text.empty()) {
      cv::putText(c_frame,
                  s_text,
                  cv::Point2f(s_tag.Center.first + 10, s_tag.Center.second + 10),
                  cv::FONT_HERSHEY_SIMPLEX,
                  1,
                  cv::Scalar(0,0,0),
                  2);
   }
}

/****************************************/
/****************************************/
#include <iomanip>

void CFrameAnnotator::Annotate(cv::Mat& c_frame,
                               const STarget& s_target,
                               const cv::Matx33f& c_camera_matrix,
                               const cv::Vec4f& c_distortion_parameters,
                               const std::string& s_text,
                               const cv::Scalar& c_color) {
                               //const std::chrono::time_point<std::chrono::steady_clock>& t_reference_time) {

   /*
   std::cerr << "Target #" << static_cast<int>(s_target.Id) << std::endl;   
   std::cerr << "s_target.PseudoObservations.size():" << s_target.PseudoObservations.size() << std::endl;
   for(const SBlock& s_block : s_target.PseudoObservations) {
      double fTimestamp = std::chrono::duration<double, std::milli>(s_block.Timestamp - t_reference_time).count();
      std::cerr << '\t' << fTimestamp << ": (" << std::setw(4) << s_block.Translation.X << ", " << s_block.Translation.Y << ", " << s_block.Translation.Z << ")" << std::endl;
   }
   std::cerr << "s_target.Observations.size():" << s_target.Observations.size() << std::endl;
   for(const SBlock& s_block : s_target.Observations) {
      double fTimestamp = std::chrono::duration<double, std::milli>(s_block.Timestamp - t_reference_time).count();
      std::cerr << '\t' << fTimestamp << ": (" << std::setw(4) << s_block.Translation.X << ", " << s_block.Translation.Y << ", " << s_block.Translation.Z << ")" << std::endl;
   }
   */

   SBlock sBlock = s_target.Observations.front();
   if(s_target.PseudoObservations.size() != 0) {
      const SBlock& sPseudoBlock = s_target.PseudoObservations.front();
      double pfPesudoTranslation[] = {sPseudoBlock.Translation.X, sPseudoBlock.Translation.Y, sPseudoBlock.Translation.Z};
      sBlock.TranslationVector = cv::Mat(3, 1, CV_32F, pfPesudoTranslation);
      sBlock.Tags.clear();
      sBlock.HackTags.clear();

      Annotate(c_frame,
               sBlock,
               c_camera_matrix,
               c_distortion_parameters,
               "",
               c_color,
               false);

   }
   else {
      Annotate(c_frame,
               sBlock,
               c_camera_matrix,
               c_distortion_parameters,
               "",
               c_color,
               false);
   }
           
   for(auto it_block = std::begin(s_target.Observations);
       it_block != std::end(s_target.Observations);
       it_block++) {

      argos::CVector3 cBlockRotation;
      argos::CRadians cBlockRotationMagnitude;

      it_block->Tags[0].Rotation.ToAngleAxis(cBlockRotationMagnitude, cBlockRotation);

      cBlockRotation *= cBlockRotationMagnitude.GetValue();

      //std::cout << "cBlockRotationMagnitude.GetValue() = " << cBlockRotationMagnitude.GetValue() << std::endl;
      //std::cout << "cBlockRotation.Length() = " << cBlockRotation.Length() << std::endl;
      //std::cout << "---" << std::endl;
      //std::cout << "ARGoS3 Rotation = " << cBlockRotation.GetX() << ", " << cBlockRotation.GetY() << ", " << cBlockRotation.GetZ() << std::endl;
      //std::cout << "OpenCV Rotation = " << it_block->RotationVector.at<double>(0, 0) << ", " << it_block->RotationVector.at<double>(1, 0) << ", " << it_block->RotationVector.at<double>(2, 0) << std::endl;

      //std::cout << "ARGoS3 Rotation = " << cBlockRotation.GetX() << ", " << cBlockRotation.GetY() << ", " << cBlockRotation.GetZ() << std::endl;

      argos::CRotationMatrix3 cRotMatrix(it_block->Tags[0].Rotation);
      cv::Mat cRotMatrixCV(3,3, CV_64F, &cRotMatrix(0,0));
      cv::Mat cRotVectorCV;
      cv::Rodrigues(cRotMatrixCV, cRotVectorCV);
      
      //std::cout << "ARGoS Rotation = " << cRotVectorCV.at<double>(0, 0) << ", " << cRotVectorCV.at<double>(1, 0) << ", " << cRotVectorCV.at<double>(2, 0) << std::endl;

      //std::cout << "OpenCV Rotation = " << it_block->Tags[0].RotationVector.at<double>(0, 0) << ", " << it_block->Tags[0].RotationVector.at<double>(1, 0) << ", " << it_block->Tags[0].RotationVector.at<double>(2, 0) << std::endl;


      //std::cout << "ARGoS3 Translation = " << cBlockRotation.GetX() << ", " << cBlockRotation.GetY() << ", " << cBlockRotation.GetZ() << std::endl;
      //std::cout << "OpenCV Translation = " << it_block->TranslationVector.at<double>(0, 0) << ", " << it_block->RotationVector.at<double>(1, 0) << ", " << it_block->RotationVector.at<double>(2, 0) << std::endl;

      


      std::vector<cv::Point3f> vecBlockCentrePoint = {
         cv::Point3f(0,0,0)
      };
      std::vector<cv::Point2f> vecBlockCentrePixel;

      cv::projectPoints(vecBlockCentrePoint,
                        //it_block->RotationVector, it_block->TranslationVector,
                        cv::Matx31d(cBlockRotation.GetX(), cBlockRotation.GetY(), cBlockRotation.GetZ()),
                        cv::Matx31d(it_block->TranslationT.GetX(), it_block->TranslationT.GetY(), it_block->TranslationT.GetZ()),
                        c_camera_matrix,
                        c_distortion_parameters,
                        vecBlockCentrePixel);


      cv::circle(c_frame, 
                 vecBlockCentrePixel[0],
                 5.0f,
                 cv::Scalar(0,0,255));

      std::list<SBlock>::const_iterator itNextBlock = std::next(it_block);
      /*         
      if(itNextBlock != std::end(s_target.Observations)) {
         cv::line(c_frame,
                  cv::Point2f(it_block->Coordinates.first, it_block->Coordinates.second),
                  cv::Point2f(itNextBlock->Coordinates.first, itNextBlock->Coordinates.second),
                  cv::Scalar(0,0,255),
                  1);
      }
      */
   }
   
   if(!s_text.empty()) {
      /*
      cv::putText(c_frame,
                  s_text,
                  cv::Point2f(std::begin(s_target.Observations)->Coordinates.first + 10,
                              std::begin(s_target.Observations)->Coordinates.second + 10),
                  cv::FONT_HERSHEY_SIMPLEX,
                  0.5,
                  cv::Scalar(255,255,255),
                  1);
     */
      /*
      Label(c_frame, cv::Point2f(std::begin(s_target.Observations)->Coordinates.first + 10,
                              std::begin(s_target.Observations)->Coordinates.second + 10), s_text);

      */
   }  
}

/****************************************/
/****************************************/

void CFrameAnnotator::Label(cv::Mat& c_frame,
                            const cv::Point2f& c_origin,
                            const std::string& str_text) {

   auto tFont = cv::FONT_HERSHEY_SIMPLEX;
   double fScale = 0.5;
   int nThickness = 1;
   int nBaseline = 0;
   int nBorderThickness = 5;

   cv::Size cTextSize = cv::getTextSize(str_text, tFont, fScale, nThickness, &nBaseline);
   cv::rectangle(c_frame,
                 c_origin + cv::Point2f(0, nBaseline) + cv::Point2f(-nBorderThickness, nBorderThickness),
                 c_origin + cv::Point2f(cTextSize.width, -cTextSize.height) + cv::Point2f(nBorderThickness, -nBorderThickness),
                 cv::Scalar(0,0,0),
                 -1);
   cv::putText(c_frame, str_text, c_origin, tFont, fScale, cv::Scalar(255,255,255), nThickness, 8);
}


