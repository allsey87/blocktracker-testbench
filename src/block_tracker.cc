
#include "block_tracker.h"
#include "hungarian_solver.h"

#include <iostream>

void CBlockTracker::AssociateAndTrackTargets(std::chrono::time_point<std::chrono::steady_clock> t_timestamp,
                                             std::list<SBlock>& lst_unassociated_blocks,
                                             std::list<STarget>& lst_targets) {
   /* write the timestamps into each entry of lst_unassociated_blocks */
   for(SBlock& s_block : lst_unassociated_blocks) {
      s_block.Timestamp = t_timestamp;
   }
   
   /* if there are existing targets, estimate their position based */
   if(lst_targets.size() > 0u) {
      /* estimate the velocity of the camera based (weighted towards more recent observations) */
      float fCameraVelocityEstimateX = 0.0f, fCameraVelocityEstimateY = 0.0f, fCameraVelocityEstimateZ = 0.0f;
      float fCameraVelocityEstimateWeight = 0.0f;
      /* calculate the current camera velocity */
      unsigned int unUsedTargetCount = 0u;
      for(STarget& s_target : lst_targets) {
         if(s_target.Observations.size() >= 2u) {
            auto itMostRecentObservation = std::begin(s_target.Observations);
            auto itSecondMostRecentObservation = std::next(itMostRecentObservation);
            float fDelta = std::chrono::duration<float>(t_timestamp - itSecondMostRecentObservation->Timestamp).count();
            float fWeight = 1.0f / fDelta;
            unUsedTargetCount++;
            fCameraVelocityEstimateWeight += fWeight;
            fCameraVelocityEstimateX += ((itMostRecentObservation->Translation.X - itSecondMostRecentObservation->Translation.X) / fDelta) * fWeight;
            fCameraVelocityEstimateY += ((itMostRecentObservation->Translation.Y - itSecondMostRecentObservation->Translation.Y) / fDelta) * fWeight;
            fCameraVelocityEstimateZ += ((itMostRecentObservation->Translation.Z - itSecondMostRecentObservation->Translation.Z) / fDelta) * fWeight;
         }
      }
      /* update the average camera velocity, if the weight is zero, we don't have enough readings to determine the camera velocity */
      if(fCameraVelocityEstimateWeight == 0.0f) {
         fCameraVelocityEstimateX = 0.0f;
         fCameraVelocityEstimateY = 0.0f;
         fCameraVelocityEstimateZ = 0.0f;
      }
      else {
         fCameraVelocityEstimateX /= fCameraVelocityEstimateWeight;
         fCameraVelocityEstimateY /= fCameraVelocityEstimateWeight;
         fCameraVelocityEstimateZ /= fCameraVelocityEstimateWeight;
      }

      /* calculate the camera velocity estimate average weight - indicates the quality (recentness) of the estimate */
      float fCameraVelocityEstimateAverageWeight = 0.0f;
      if(unUsedTargetCount != 0) {
         fCameraVelocityEstimateAverageWeight = fCameraVelocityEstimateWeight / static_cast<float>(unUsedTargetCount);
      }

      /* estimate velocities of existing targets, storing the result as a pseudo observation */
      for(STarget& s_target : lst_targets) {
         /* initialise the estimates to the camera velocity */
         float fTargetVelocityEstimateX = fCameraVelocityEstimateX * fCameraVelocityEstimateAverageWeight;
         float fTargetVelocityEstimateY = fCameraVelocityEstimateY * fCameraVelocityEstimateAverageWeight;
         float fTargetVelocityEstimateZ = fCameraVelocityEstimateZ * fCameraVelocityEstimateAverageWeight;

         auto itMostRecentObservation = std::begin(s_target.Observations);
         float fTargetVelocityWeight = 0.0f;
         /* if there are two or more observations, compute an average velocity of the target */
         if(s_target.Observations.size() >= 2u) {
            auto itSecondMostRecentObservation = std::next(itMostRecentObservation);
            float fDelta = std::chrono::duration<float>(itMostRecentObservation->Timestamp - itSecondMostRecentObservation->Timestamp).count();
            /* calculate the weight from delta */
            fTargetVelocityWeight = 1.0f / fDelta;
            fTargetVelocityEstimateX += ((itMostRecentObservation->Translation.X - itSecondMostRecentObservation->Translation.X) / fDelta) * fTargetVelocityWeight;
            fTargetVelocityEstimateY += ((itMostRecentObservation->Translation.Y - itSecondMostRecentObservation->Translation.Y) / fDelta) * fTargetVelocityWeight;
            fTargetVelocityEstimateZ += ((itMostRecentObservation->Translation.Z - itSecondMostRecentObservation->Translation.Z) / fDelta) * fTargetVelocityWeight;
         }
         /* calculate the target velocity estimate */
         float fTotalWeight = fCameraVelocityEstimateAverageWeight + fTargetVelocityWeight;
         if(fTotalWeight == 0.0f) {
            fTargetVelocityEstimateX = 0.0f;
            fTargetVelocityEstimateY = 0.0f;
            fTargetVelocityEstimateZ = 0.0f;
         }
         else {
            fTargetVelocityEstimateX /= fTotalWeight;
            fTargetVelocityEstimateY /= fTotalWeight;
            fTargetVelocityEstimateZ /= fTotalWeight;
         }
         /* Calculate the expected position of the target */
         float fElapsedTime = std::chrono::duration<float>(t_timestamp - itMostRecentObservation->Timestamp).count();
         float fTargetPseudoPositionX = itMostRecentObservation->Translation.X + fTargetVelocityEstimateX * fElapsedTime;
         float fTargetPseudoPositionY = itMostRecentObservation->Translation.Y + fTargetVelocityEstimateY * fElapsedTime;
         float fTargetPseudoPositionZ = itMostRecentObservation->Translation.Z + fTargetVelocityEstimateZ * fElapsedTime;
         /* Add a pseudo observation using the default constructor */
         s_target.PseudoObservations.emplace_front();
         s_target.PseudoObservations.front().Translation.X = fTargetPseudoPositionX;
         s_target.PseudoObservations.front().Translation.Y = fTargetPseudoPositionY;
         s_target.PseudoObservations.front().Translation.Z = fTargetPseudoPositionZ;
         s_target.PseudoObservations.front().Timestamp = t_timestamp;
      }
   }

   /* list of blocks that are don't correspond to any target */
   std::list<SBlock> lstUnmatchedBlocks;
   std::list<STarget> lstUnmatchedTargets;
   std::list<STarget> lstNewTargets;

   /* build a cost matrix */
   CHungarianSolver::TCostMatrix tCostMatrix(lst_unassociated_blocks.size(), std::vector<double>(lst_targets.size(), 0.0f));
   unsigned int n_target_idx = 0u;
   auto it_target = std::begin(lst_targets);
   for(; it_target != std::end(lst_targets); n_target_idx++, it_target++) {
      unsigned int n_block_idx = 0u;
      auto it_block = std::begin(lst_unassociated_blocks);
      for(; it_block != std::end(lst_unassociated_blocks); n_block_idx++, it_block++) {
         // Cost matrix is blocks x targets
         const SBlock& sTrackedBlock = it_target->PseudoObservations.front();

         tCostMatrix[n_block_idx][n_target_idx] = std::sqrt(std::pow(sTrackedBlock.Translation.X - it_block->Translation.X, 2) +
                                                            std::pow(sTrackedBlock.Translation.Y - it_block->Translation.Y, 2) +
                                                            std::pow(sTrackedBlock.Translation.Z - it_block->Translation.Z, 2));
      }
   }

   /* remove rows from the cost matrix that exceed the distance threshold */
   std::list<unsigned int> lstRowsIndicesToRemove;
   for(auto it_row = std::begin(tCostMatrix); it_row != std::end(tCostMatrix); it_row++) {
      bool bRowExceedsThreshold = true;
      for(auto it_el = std::begin(*it_row); it_el != std::end(*it_row); it_el++) {
         bRowExceedsThreshold = bRowExceedsThreshold && (*it_el > m_fDistanceThreshold);
      }
      if(bRowExceedsThreshold) {
         /* add this row to the list for removal */
         lstRowsIndicesToRemove.push_front(std::distance(std::begin(tCostMatrix), it_row));
      }
   }
   /* remove rows / move unmatched blocks to unmatched list  */
   for(unsigned int un_idx : lstRowsIndicesToRemove) {
      auto it_erase = std::begin(tCostMatrix);
      std::advance(it_erase, un_idx);
      tCostMatrix.erase(it_erase);
      /* move the unassociated block into the unmatched list */
      auto it_move = std::begin(lst_unassociated_blocks);
      std::advance(it_move, un_idx);
      lstUnmatchedBlocks.splice(std::begin(lstUnmatchedBlocks), lst_unassociated_blocks, it_move);
   }
   
   /* remove columns from the cost matrix that exceed the distance threshold */
   /* check if matrix has at least 1 row */
   if(tCostMatrix.size() > 0) {
      unsigned int unColumnCount = tCostMatrix[0].size();
      std::list<unsigned int> lstColumnIndicesToRemove;
      for(unsigned int un_col = 0; un_col < unColumnCount; un_col++) {
         bool bColExceedsThreshold = true;
         for(auto it_row = std::begin(tCostMatrix); it_row != std::end(tCostMatrix); it_row++) {
            auto it_el = std::begin(*it_row);     
            std::advance(it_el, un_col);
            bColExceedsThreshold = bColExceedsThreshold && (*it_el > m_fDistanceThreshold);
         }
         if(bColExceedsThreshold) {
            lstColumnIndicesToRemove.push_front(un_col);
         }
      }
      for(auto un_idx : lstColumnIndicesToRemove) {
         for(auto it_row = std::begin(tCostMatrix); it_row != std::end(tCostMatrix); it_row++) {
            auto it_col_el = std::begin(*it_row);
            std::advance(it_col_el, un_idx);
            it_row->erase(it_col_el);
         }
         /* move unassociated target into the unmatched list */
         auto it_move = std::begin(lst_targets);
         std::advance(it_move, un_idx);
         lstUnmatchedTargets.splice(std::begin(lstUnmatchedTargets), lst_targets, it_move);
      }
   }      

   /* adjust individual elements that exceed m_fDistanceThreshold */
   for(auto it_row = std::begin(tCostMatrix); it_row != std::end(tCostMatrix); it_row++) {
      for(auto it_el = std::begin(*it_row); it_el != std::end(*it_row); it_el++) {
         *it_el = (*it_el < m_fDistanceThreshold) ? *it_el : std::max(lst_targets.size(), lst_unassociated_blocks.size()) * m_fDistanceThreshold;
      }
   }

   /* zero pad the cost matrix until it is square */
   unsigned int unRowCount = tCostMatrix.size();
   unsigned int unColCount = (unRowCount > 0) ? tCostMatrix[0].size() : 0;

   if(unRowCount > unColCount) {
      /* add columns */
      for(auto it_row = std::begin(tCostMatrix); it_row != std::end(tCostMatrix); it_row++) {
         it_row->insert(std::end(*it_row), unRowCount - unColCount, 0.0f);
      }
   }

   if(unColCount > unRowCount) {
      /* add rows */
      tCostMatrix.insert(std::end(tCostMatrix), unColCount - unRowCount, std::vector<double>(unColCount, 0.0f));
   }

   /* negate all elements such that we solve for the minimum */
   for(auto it_row = std::begin(tCostMatrix); it_row != std::end(tCostMatrix); it_row++) {
      for(auto it_el = std::begin(*it_row); it_el != std::end(*it_row); it_el++) {
         *it_el = -(*it_el);
      }
   }

   /* create instance of the solver */
   CHungarianSolver CHungarianSolver;
   /* run the solver on the cost matrix */   
   CHungarianSolver(tCostMatrix);
   /* get the indices of the blocks and targets to be assigned */
   std::vector<std::pair<int, int> > vecAssignmentIndices = CHungarianSolver.GetAssignments();

   /* schedule all */
   std::list<std::pair<std::list<SBlock>::iterator, std::list<STarget>::iterator> > lstScheduledAssignments;

   for(const auto& t_assignment : vecAssignmentIndices) {
      bool bBlockExists = (t_assignment.first < lst_unassociated_blocks.size());
      bool bTargetExists = (t_assignment.second < lst_targets.size());

      if(bBlockExists) {
         auto itBlock = std::begin(lst_unassociated_blocks);
         std::advance(itBlock, t_assignment.first);
         if(bTargetExists) {
            auto itTarget = std::begin(lst_targets);
            std::advance(itTarget, t_assignment.second);
            // clear itTarget->pseudoObservations ??
            lstScheduledAssignments.push_front(std::make_pair(itBlock, itTarget));
         }
         else  {
            /* create a new target */
            lstNewTargets.emplace_front();
            /* schedule the assignment */
            lstScheduledAssignments.push_front(std::make_pair(itBlock, std::begin(lstNewTargets)));
         }
      }
      else {
         // !bBlockExists
         // do nothing? PseudoTargets already increased due to first part of algorithm
      }
   }

   /* do the assignments */
   for(auto& c_scheduled_assignment : lstScheduledAssignments) {
      std::list<SBlock>& sObservationList = c_scheduled_assignment.second->Observations;
      sObservationList.splice(std::begin(sObservationList), lst_unassociated_blocks, c_scheduled_assignment.first);
      /* clear pseudo observations */
      c_scheduled_assignment.second->PseudoObservations.clear();
      /* limit the number of observations to m_unTrackingDepth */
      while(c_scheduled_assignment.second->Observations.size() > m_unTrackingDepth) {
         c_scheduled_assignment.second->Observations.pop_back();
      }
   }

   //lstUnmatchedBlocks - create targets and assign
   while(!lstUnmatchedBlocks.empty()) {
      auto itUnmatchedBlock = std::begin(lstUnmatchedBlocks);
      lstNewTargets.emplace_front();
      std::list<SBlock>& sObservationList = std::begin(lstNewTargets)->Observations;
      sObservationList.splice(std::begin(sObservationList), lstUnmatchedBlocks, itUnmatchedBlock);  
   }

   /* move all targets back to main list */
   lst_targets.splice(std::begin(lst_targets), lstNewTargets);
   lst_targets.splice(std::begin(lst_targets), lstUnmatchedTargets);
   
   /* clear targets with pseudo count higher than tracking threshold */
   lst_targets.remove_if([this] (const STarget& s_target) {
      return (s_target.PseudoObservations.size() > m_unTrackingDepth);
   });

   /* assign indentifiers */
   AssignIdentifiers(lst_targets);
}

void CBlockTracker::AssignIdentifiers(std::list<STarget>& lst_targets) {
   /* create a list of the used target indentifiers */
   std::list<unsigned int> lstUsedIds;
   /* populate that list */
   for(STarget& s_target : lst_targets) {
      if(s_target.Id != -1) {
         lstUsedIds.push_back(s_target.Id);
      }
   }
   /* assign identifiers */
   for(STarget& s_target : lst_targets) {
      if(s_target.Id == -1) {
         while((m_unNextId == -1) || std::find(std::begin(lstUsedIds), std::end(lstUsedIds), m_unNextId) != std::end(lstUsedIds)) {
            m_unNextId++;
         }
         s_target.Id = m_unNextId;
         lstUsedIds.push_back(m_unNextId);
      }
   }
}
