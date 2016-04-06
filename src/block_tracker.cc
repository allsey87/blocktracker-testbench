
#include "block_tracker.h"

#include <iostream>


   /* 
      if issues are occuring with blocks close to the robot moving faster (i.e. due to spot turn /
      distance from robot) adjust the average velocity taking the distance of the block from
      the robot into consideration (is it possible to estimate the angular/linear velocity of
      the robot?) - possibly complicated! avoid if possible.
   */



void CBlockTracker::AssociateAndTrackTargets(std::chrono::time_point<std::chrono::steady_clock> t_timestamp,
                                             std::list<SBlock>& lst_unassociated_blocks,
                                             std::list<STarget>& lst_targets) {
   /* write the timestamps into each entry of lst_unassociated_blocks */
   for(SBlock& s_block : lst_unassociated_blocks) {
      s_block.Timestamp = t_timestamp;
   }
   /* predict velocities of targets, storing results as pseudo observations */
   if(lst_targets.size() > 0u) {
      /* estimates for the current camera velocity */
      float fCameraVelocityEstimateX = 0.0f, fCameraVelocityEstimateY = 0.0f, fCameraVelocityEstimateZ = 0.0f;
      float fCameraVelocityEstimateWeight = 0.0f;
      /* calculate the current camera velocity */
      for(STarget& s_target : lst_targets) {
         if(s_target.Observations.size() > 1u) {
            auto itMostRecentObservation = std::begin(s_target.Observations);
            auto itSecondMostRecentObservation = std::next(itMostRecentObservation);
            float fDelta = std::chrono::duration<float>(t_timestamp - itSecondMostRecentObservation->Timestamp).count();
            float fWeight = 1.0f / fDelta;
            fCameraVelocityEstimateWeight += fWeight;
            fCameraVelocityEstimateX += ((itMostRecentObservation->Translation.X - itSecondMostRecentObservation->Translation.X) / fDelta) * fWeight;
            fCameraVelocityEstimateY += ((itMostRecentObservation->Translation.Y - itSecondMostRecentObservation->Translation.Y) / fDelta) * fWeight;
            fCameraVelocityEstimateZ += ((itMostRecentObservation->Translation.Z - itSecondMostRecentObservation->Translation.Z) / fDelta) * fWeight;
         }
      }
      /* update the average camera velocity */
      if(fCameraVelocityEstimateWeight != 0.0f) {
         fCameraVelocityEstimateX /= fCameraVelocityEstimateWeight;
         fCameraVelocityEstimateY /= fCameraVelocityEstimateWeight;
         fCameraVelocityEstimateZ /= fCameraVelocityEstimateWeight;
      }
      float fCameraVelocityEstimateAverageWeight = fCameraVelocityEstimateWeight / static_cast<float>(lst_targets.size());
      /* estimate the velocities of the targets and create the observation */
      for(STarget& s_target : lst_targets) {
         auto itMostRecentObservation = std::begin(s_target.Observations);

         float fTargetVelocityEstimateX = fCameraVelocityEstimateX * fCameraVelocityEstimateAverageWeight;
         float fTargetVelocityEstimateY = fCameraVelocityEstimateY * fCameraVelocityEstimateAverageWeight;
         float fTargetVelocityEstimateZ = fCameraVelocityEstimateZ * fCameraVelocityEstimateAverageWeight;
         float fTargetVelocityWeight = 0.0f;

         /* if there are two or more observations, compute an average velocity of the target */
         if(s_target.Observations.size() > 1u) {
            auto itSecondMostRecentObservation = std::next(itMostRecentObservation);
            float fDelta = std::chrono::duration<float>(itMostRecentObservation->Timestamp - itSecondMostRecentObservation->Timestamp).count();
            /* calculate the weight from delta */
            fTargetVelocityWeight = 1.0f / fDelta;
            fTargetVelocityEstimateX += ((itMostRecentObservation->Translation.X - itSecondMostRecentObservation->Translation.X) / fDelta) * fTargetVelocityWeight;
            fTargetVelocityEstimateY += ((itMostRecentObservation->Translation.Y - itSecondMostRecentObservation->Translation.Y) / fDelta) * fTargetVelocityWeight;
            fTargetVelocityEstimateZ += ((itMostRecentObservation->Translation.Z - itSecondMostRecentObservation->Translation.Z) / fDelta) * fTargetVelocityWeight;
         }
         /* calculate the target velocity estimate */
         if((fCameraVelocityEstimateAverageWeight + fTargetVelocityWeight) != 0.0f) {
            fTargetVelocityEstimateX /= (fCameraVelocityEstimateAverageWeight + fTargetVelocityWeight);
            fTargetVelocityEstimateY /= (fCameraVelocityEstimateAverageWeight + fTargetVelocityWeight);
            fTargetVelocityEstimateZ /= (fCameraVelocityEstimateAverageWeight + fTargetVelocityWeight);
         }
         /* Calculate the expected position of the target */
         float fElapsedTime = std::chrono::duration<float>(t_timestamp - itMostRecentObservation->Timestamp).count();
         float fTargetPseudoPositionX = itMostRecentObservation->Translation.X + fTargetVelocityEstimateX * fElapsedTime;
         float fTargetPseudoPositionY = itMostRecentObservation->Translation.Y + fTargetVelocityEstimateY * fElapsedTime;
         float fTargetPseudoPositionZ = itMostRecentObservation->Translation.Z + fTargetVelocityEstimateZ * fElapsedTime;
         /* Add a pseudo observation using the default constructor */
         s_target.Observations.emplace_front();
         s_target.Observations.front().Translation.X = fTargetPseudoPositionX;
         s_target.Observations.front().Translation.Y = fTargetPseudoPositionY;
         s_target.Observations.front().Translation.Z = fTargetPseudoPositionZ;

         s_target.Observations.front().RotationVector = itMostRecentObservation->RotationVector;
         s_target.Observations.front().TranslationVector = itMostRecentObservation->TranslationVector;
         s_target.Observations.front().Timestamp = t_timestamp;

         s_target.Observations.front().IsPseudo = true;
      }
   }

   /* list for keeping track of associations between the blocks and the existing targets */
   std::list<SAssociation> lstAssociations, lstFinalAssociations;
   /* while we still have unassociated blocks or targets */
   while(!lst_targets.empty() || !lst_unassociated_blocks.empty()) {
      /* for each detected block */
      while(!lst_unassociated_blocks.empty()) {
         /* Take the first block in the list */
         std::list<SBlock>::iterator itUnassociatedBlock = std::begin(lst_unassociated_blocks);
         /* compare the block to each target, keeping track of the target with the
            minimum distance between itself and the block */
         std::list<STarget>::iterator itClosestTrackedTarget = std::end(lst_targets);
         float fClosestTrackedTargetDist = 0;
         /* select the closest target to the block */
         for(std::list<STarget>::iterator itTrackedTarget = std::begin(lst_targets);
             itTrackedTarget != std::end(lst_targets);
             itTrackedTarget++) {
            std::list<SBlock>::iterator itTrackedBlock = std::begin(itTrackedTarget->Observations);
            /* calculate the distance between this block and the tracked block */
            float fInterblockDist =
               sqrt(pow(itTrackedBlock->Translation.X - itUnassociatedBlock->Translation.X, 2) +
                    pow(itTrackedBlock->Translation.Y - itUnassociatedBlock->Translation.Y, 2) +
                    pow(itTrackedBlock->Translation.Z - itUnassociatedBlock->Translation.Z, 2));
            if(itClosestTrackedTarget == std::end(lst_targets) ||
               fInterblockDist < fClosestTrackedTargetDist) {
               itClosestTrackedTarget = itTrackedTarget;
               fClosestTrackedTargetDist = fInterblockDist;
            }
         }
         /* consider the case of a new block appearing (i.e. a new target) */
         float fMinDistToFrame = CalculateMinimumDistanceToFrame(itUnassociatedBlock->Coordinates);
         float fNewTrackedTargetDist = m_fPixelToDistanceCoefficient / (fMinDistToFrame + 1);
         if(itClosestTrackedTarget == std::end(lst_targets) ||
            fNewTrackedTargetDist < fClosestTrackedTargetDist) {
            /* create a new association with */
            lstAssociations.emplace_back(fNewTrackedTargetDist);
            /* reference the list for the candidate block */
            std::list<SBlock>& lstCandidateBlock = lstAssociations.back().CandidateBlock;
            /* move the block into this list */
            lstCandidateBlock.splice(std::begin(lstCandidateBlock),
                                     lst_unassociated_blocks,
                                     itUnassociatedBlock);
         }
         else {
            /* set the association distance for the closest tracked target */
            lstAssociations.emplace_back(fClosestTrackedTargetDist);
            /* reference the list for the candidate block and existing target */
            std::list<STarget>& lstExistingTarget = lstAssociations.back().ExistingTarget;
            std::list<SBlock>& lstCandidateBlock = lstAssociations.back().CandidateBlock;
            /* move the existing target and block into the association */
            lstExistingTarget.splice(std::begin(lstExistingTarget),
                                     lst_targets,
                                     itClosestTrackedTarget);
            lstCandidateBlock.splice(std::begin(lstCandidateBlock),
                                     lst_unassociated_blocks,
                                     itUnassociatedBlock);
         }
      } /* while(!lst_unassociated_blocks.empty()) */
      /* For each target that doesn't have an associated block */
      while(!lst_targets.empty()) {
         std::list<STarget>::iterator itLostTarget = std::begin(lst_targets);
         /* compute an effective tracking distance based on the distance to the frame */
         float fMinDistToFrame =
            CalculateMinimumDistanceToFrame(std::begin(itLostTarget->Observations)->Coordinates);
         lstAssociations.emplace_back(m_fPixelToDistanceCoefficient / (fMinDistToFrame + 1));
         /* reference the list for the existing target */
         std::list<STarget>& lstExistingTarget = lstAssociations.back().ExistingTarget;
         /* move the existing target into the association */
         lstExistingTarget.splice(std::begin(lstExistingTarget),
                                  lst_targets,
                                  itLostTarget);
      }
      /* sort the list of associations by distance */
      lstAssociations.sort([] (const SAssociation& s_association_first,
                               const SAssociation& s_association_second) {
                              return (s_association_first.AssociationDist <
                                      s_association_second.AssociationDist);
                           });
      /* determine the range of the X best associations */
      std::list<SAssociation>::iterator itFinalAssociationRangeEnd = std::begin(lstAssociations);
      std::advance(itFinalAssociationRangeEnd,
                   std::ceil(lstAssociations.size() * (1 - m_fAssociationRecursionRatio)));
      /* move the X best associations into the final associations list */
      lstFinalAssociations.splice(std::begin(lstFinalAssociations),
                                  lstAssociations,
                                  std::begin(lstAssociations),
                                  itFinalAssociationRangeEnd);
      /* undo the remaining associations */
      for(SAssociation& s_association : lstAssociations) {
         if(!s_association.ExistingTarget.empty()) {
            lst_targets.splice(std::begin(lst_targets), s_association.ExistingTarget);
         }
         if(!s_association.CandidateBlock.empty()) {
            lst_unassociated_blocks.splice(std::begin(lst_unassociated_blocks), s_association.CandidateBlock);
         }
      }
      lstAssociations.clear();
   } /* while(!lst_targets.empty() || !lst_unassociated_blocks.empty()) */
   


   /* move the final associations back into the target list */
   for(SAssociation& s_association : lstFinalAssociations) {
      /* if association doesn't have a target, create one */
      if(s_association.ExistingTarget.empty()) {
         s_association.ExistingTarget.emplace_back();
      }
      /* if a candidate block exists, move it into the target observations list */
      if(!s_association.CandidateBlock.empty()) {
         std::list<SBlock>& lstObservations = std::begin(s_association.ExistingTarget)->Observations;
         /* this is an actual observation, remove any pseudo observations */
         lstObservations.remove_if([](const SBlock& s_block) { return s_block.IsPseudo; });
         /* reset the frames since last observation counter */
         std::begin(s_association.ExistingTarget)->FramesSinceLastObservation = 0;           
         /* add the observation */
         lstObservations.splice(std::begin(lstObservations), s_association.CandidateBlock);
         /* limit the number of observations to m_unTrackingDepth */
         while(lstObservations.size() > m_unTrackingDepth) {
            lstObservations.pop_back();
         }
      }
      else {
         std::begin(s_association.ExistingTarget)->FramesSinceLastObservation++;   
      }
      /* if we haven't exceeded m_unLostTargetThreshold, move the target back into the targets list */
      if(std::begin(s_association.ExistingTarget)->FramesSinceLastObservation < m_unLostTargetThreshold) {
         lst_targets.splice(std::begin(lst_targets), s_association.ExistingTarget);
      }
   }

   AssignIdentifiers(lst_targets);
}

void CBlockTracker::AssignIdentifiers(std::list<STarget>& lst_targets) {
   /* create a list of the used target indentifiers */
   std::list<unsigned int> lstUsedIds;
   unsigned int unNextId = 0;
   /* populate that list */
   for(STarget& s_target : lst_targets) {
      if(s_target.Id != -1) {
         lstUsedIds.push_back(s_target.Id);
      }
   }
   /* assign identifiers */
   for(STarget& s_target : lst_targets) {
      if(s_target.Id == -1) {
         while(std::find(std::begin(lstUsedIds), std::end(lstUsedIds), unNextId) != std::end(lstUsedIds)) {
            unNextId++;
         }
         s_target.Id = unNextId;
         lstUsedIds.push_back(unNextId);
      }
   }
}


float CBlockTracker::CalculateMinimumDistanceToFrame(const std::pair<float, float>& c_coordinates) {
   /* init fMinDistToFrame as float max */
   float fMinDistToFrame = std::numeric_limits<float>::max();
   float fDistToFrames[] = {
      c_coordinates.second,
      c_coordinates.first,
      m_unFrameHeight - c_coordinates.second,
      m_unFrameWidth - c_coordinates.first
   };
   /* calculate the smallest distance from frame */
   for(float fDist : fDistToFrames) {
      /* saturate at zero, c_coordinates could be outside of the frame for velocity based matching */
      if(fDist < 0) {
         fMinDistToFrame = 0;
         break;
      }
      else if(fDist < fMinDistToFrame) {
         fMinDistToFrame = fDist;
      }
   }
   return fMinDistToFrame;
}
