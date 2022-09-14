#ifndef _PATH_FOLLOW_H
#define  _PATH_FOLLOW_H
#include "Eigen/Core"

class PathFollowInterfase {
 public:
  virtual Eigen::Vector2d ComputeVelocity() = 0;
  virtual void Cancel() = 0;
  virtual ~PathFollowInterfase() {}
};

#endif