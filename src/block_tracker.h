#ifndef BLOCK_TRACKER_H
#define BLOCK_TRACKER_H

#include <list>
#include <algorithm>

#include "block.h"
#include "target.h"

class CBlockTracker {

public:
   CBlockTracker(unsigned int un_frame_width,
                 unsigned int un_frame_height,
                 unsigned int un_tracking_depth,
                 float f_pixel_to_distance_coefficient) :

      m_unFrameWidth(un_frame_width),
      m_unFrameHeight(un_frame_height),
      m_unTrackingDepth(un_tracking_depth),
      m_fPixelToDistanceCoefficient(f_pixel_to_distance_coefficient) {}

   void AssociateAndTrackTargets(std::chrono::time_point<std::chrono::steady_clock> t_timestamp,
                                 std::list<SBlock>& lst_unassociated_blocks,
                                 std::list<STarget>& lst_targets);
private:

   struct SAssociation {
      SAssociation(float f_association_distance) :
         Distance(f_association_distance) {}

      SAssociation(float f_association_distance,
                   std::list<SBlock>::iterator it_block,
                   std::list<STarget>::iterator it_target) :
         Distance(f_association_distance),
         CandidateBlock(it_block),
         ExistingTarget(it_target) {}
      
      std::list<SBlock>::iterator CandidateBlock;
      std::list<STarget>::iterator ExistingTarget;
      float Distance;
   };

   void AssignIdentifiers(std::list<STarget>& lst_targets);

   //float CalculateMinimumDistanceToFrame(const std::pair<float, float>& c_coordinates);

   unsigned int m_unFrameWidth;
   unsigned int m_unFrameHeight;
   unsigned int m_unTrackingDepth;
   unsigned int m_unNextId = 0;
   float m_fPixelToDistanceCoefficient;

};

#endif
