#ifndef BLOCK_TRACKER_H
#define BLOCK_TRACKER_H

#include <list>
#include <algorithm>

#include "block.h"
#include "target.h"

class CBlockTracker {

public:
   CBlockTracker(unsigned int un_tracking_depth,
                 double f_distance_threshold) :

      m_unTrackingDepth(un_tracking_depth),
      m_fDistanceThreshold(f_distance_threshold) {}

   void AssociateAndTrackTargets(std::chrono::time_point<std::chrono::steady_clock> t_timestamp,
                                 std::list<SBlock>& lst_unassociated_blocks,
                                 std::list<STarget>& lst_targets);
private:
   void AssignIdentifiers(std::list<STarget>& lst_targets);

   unsigned int m_unTrackingDepth;
   double m_fDistanceThreshold;
   unsigned int m_unNextId = 0;
};

#endif
