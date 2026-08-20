#include "bucketedqueue.h"

#include "limits.h"
#include <stdio.h>
#include <stdlib.h>

using namespace HybridAStar;

std::vector<int> BucketPrioQueue::sqrIndices;
int BucketPrioQueue::numBuckets;


BucketPrioQueue::BucketPrioQueue() {
  // make sure the index array is created
  if (sqrIndices.size()==0) initSqrIndices();
  nextBucket = INT_MAX;

  // now create the buckets
  buckets = std::vector<std::queue<INTPOINT> >(numBuckets);

  // reset element counter
  count = 0;
}

bool BucketPrioQueue::empty() const {
  return (count==0);
}


void BucketPrioQueue::push(int prio, INTPOINT t) {
  if (prio>=(int)sqrIndices.size()) {
    fprintf(stderr, "error: priority %d is not a valid squared distance x*x+y*y, or x>MAXDIST or y>MAXDIST.\n", prio);
    exit(-1);
  }
  int id = sqrIndices[prio];
  if (id<0) {
    fprintf(stderr, "error: priority %d is not a valid squared distance x*x+y*y, or x>MAXDIST or y>MAXDIST.\n", prio);
    exit(-1);
  }
  buckets[id].push(t);
  if (id<nextBucket) nextBucket = id;
  count++;
}

INTPOINT BucketPrioQueue::pop() {
  int i;
  assert(count>0);
  for (i = nextBucket; i<(int)buckets.size(); i++) {
    if (!buckets[i].empty()) break;
  }
  assert(i<(int)buckets.size());
  nextBucket = i;
  count--;
  INTPOINT p = buckets[i].front();
  buckets[i].pop();
  return p;
}


void BucketPrioQueue::initSqrIndices() {
  sqrIndices = std::vector<int>(2*MAXDIST*MAXDIST+1, -1);

  int count=0;
  for (int x=0; x<=MAXDIST; x++) {
    for (int y=0; y<=x; y++) {
      int sqr = x*x+y*y;
      sqrIndices[sqr] = count++;
    }
  }
  numBuckets = count;
}
