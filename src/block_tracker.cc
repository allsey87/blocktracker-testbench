
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

   /* list for keeping track of associations between the blocks and the existing targets */
   std::list<SAssociation> lstCandidateAssociations, lstAssociations;
   std::list<SBlock> lstAssociatedBlocks;
   std::list<STarget> lstAssociatedTargets;

   /* while we still have unassociated blocks */
   while(!lst_unassociated_blocks.empty()) {
      /* TODO: it is required that at least one target be present in this list or an assertion must be made that itClosestTrackedTarget != std::end(...) check condition above */
      for(auto itUnassociatedBlock = std::begin(lst_unassociated_blocks);
          itUnassociatedBlock != std::end(lst_unassociated_blocks);
          itUnassociatedBlock++) {
         /* compare the block to each target, keeping track of the target with the
            minimum distance between itself and the block */
         auto itClosestTrackedTarget = std::end(lst_targets);
         float fClosestTrackedTargetDist = std::numeric_limits<float>::max();
         /* select the closest target to the block */
         for(auto itTrackedTarget = std::begin(lst_targets);
             itTrackedTarget != std::end(lst_targets);
             itTrackedTarget++) {
            auto itTrackedBlock = std::begin(itTrackedTarget->PseudoObservations);
            /* calculate the distance between this block and the tracked block */
            float fInterblockDist =
               std::sqrt(std::pow(itTrackedBlock->Translation.X - itUnassociatedBlock->Translation.X, 2) +
                         std::pow(itTrackedBlock->Translation.Y - itUnassociatedBlock->Translation.Y, 2) +
                         std::pow(itTrackedBlock->Translation.Z - itUnassociatedBlock->Translation.Z, 2));
            if(fInterblockDist < fClosestTrackedTargetDist) {
               itClosestTrackedTarget = itTrackedTarget;
               fClosestTrackedTargetDist = fInterblockDist;
            }
         } /* for each target */
         lstCandidateAssociations.emplace_back(fClosestTrackedTargetDist, itUnassociatedBlock, itClosestTrackedTarget);
      } /* for each block */
      
      /* find candidate associations without targets or targets with an association distance that exceeds the threshold */
      for(auto itAssociation = std::begin(lstCandidateAssociations);
          itAssociation != std::end(lstCandidateAssociations);
          itAssociation++) {
         /* note: if lst_targets was empty, itFirstAssociation->Distance == std::numeric_limits<float>::max() */
         if(itAssociation->Distance > 0.1f) { // 10cm
            /* create a new target */
            itAssociation->ExistingTarget = lst_targets.emplace(std::end(lst_targets));
            /* note: this is a special and an unique target created for this association. As it is unique it 
               passes through the duplicates check and is moved into the lstAssociatedTargets as to not be
               reassigned to a different block */
         }
      }
   
      /* mark duplicates for removal (keeping the better match) */
      for(auto itFirstAssociation = std::begin(lstCandidateAssociations);
          itFirstAssociation != std::end(lstCandidateAssociations);
          itFirstAssociation++) {
         for(auto itSecondAssociation = std::begin(lstCandidateAssociations);
             itSecondAssociation != std::end(lstCandidateAssociations);
             itSecondAssociation++) {
            /* don't compare candidate associations with themselves */
            if(itFirstAssociation == itSecondAssociation) {
               continue;
            }
            else {
               /* if two candidate associations have the same target */
               if(itFirstAssociation->ExistingTarget == itSecondAssociation->ExistingTarget) {
                  /* keep the association with the smallest distance */
                  if(itFirstAssociation->Distance < itSecondAssociation->Distance) {
                     itSecondAssociation->ExistingTarget = std::end(lst_targets);
                  }
                  else {
                     itFirstAssociation->ExistingTarget = std::end(lst_targets);
                  }
               }
            }
         }
      }
      /* remove the duplicates from the candidate association list */
      lstCandidateAssociations.remove_if([&lst_targets](const SAssociation& s_association) {
         return (s_association.ExistingTarget == std::end(lst_targets));
      });
      /* remove the associated blocks and targets from the unassociated lists */
      for(SAssociation& s_association : lstCandidateAssociations) {
         /* note: during these operations the iterators CandidateBlock and ExistingTarget should not be invalidated */
         lstAssociatedBlocks.splice(std::end(lstAssociatedBlocks), lst_unassociated_blocks, s_association.CandidateBlock);
         lstAssociatedTargets.splice(std::end(lstAssociatedTargets), lst_targets, s_association.ExistingTarget);
      }
      /* move the candidate associations into the association list */
      lstAssociations.splice(std::end(lstAssociations), lstCandidateAssociations);
   } /* while (!lst_unassociated_blocks.empty()) */
   
   /* rebuild the list of targets adding observations */
   for(SAssociation& s_association : lstAssociations) {
      std::list<SBlock>& lstTargetObservations = s_association.ExistingTarget->Observations;
      std::list<SBlock>& lstTargetPseudoObservations = s_association.ExistingTarget->PseudoObservations;
      /* since we have a new observation, we can clear the pseudo observations */
      lstTargetPseudoObservations.clear();
      lstTargetObservations.splice(std::begin(lstTargetObservations), lstAssociatedBlocks, s_association.CandidateBlock);
      while(lstTargetObservations.size() > m_unTrackingDepth) {
         lstTargetObservations.pop_back();
      }
   }

   /* remove targets that have had no observations for the last m_unTrackingDepth rounds */
   lst_targets.remove_if([this] (const STarget& s_target) {
      return (s_target.PseudoObservations.size() > m_unTrackingDepth);
   });

   /* move all lstAssociatedTargets back into lst_targets */
   lst_targets.splice(std::begin(lst_targets), lstAssociatedTargets);

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
