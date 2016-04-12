#ifndef TARGET_H
#define TARGET_H

#include <list>
#include <string>

#include "block.h"

struct STarget {     
   std::list<SBlock> Observations;

   std::list<SBlock> PseudoObservations;

   /* -1 represents no assigned identifier */
   unsigned int Id = -1;
};

#endif
