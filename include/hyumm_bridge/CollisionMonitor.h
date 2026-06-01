// CollisionMonitor -- binary self-collision check for the real MRT loop.
//
// Reuses ocs2_self_collision's PinocchioGeometryInterface to answer the cheap
// question the phase-clock needs: "is this 9-D state in self-collision?" It uses
// ONLY computeDistances()/min_distance (no Jacobians/gradients), so it is far
// cheaper than the SQP self-collision *constraint* -- ~1-3 ms per call, fine for
// the 100 Hz NRT loop (the design doc's "collision-stop is almost free" path).
//
// NOTE (from the design doc): computeDistances() heap-allocates a GeometryData
// every call, so this is for the 100 Hz NRT loop ONLY -- never a 1 kHz hard-RT
// thread.

#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <ocs2_core/Types.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_self_collision/PinocchioGeometryInterface.h>

namespace hyumm_bridge {

class CollisionMonitor {
 public:
  CollisionMonitor(const ocs2::PinocchioInterface& pinocchioInterface,
                   const std::vector<std::pair<std::string, std::string>>& collisionLinkPairs,
                   ocs2::scalar_t minimumDistance);

  // Build from task.info (selfCollision.collisionLinkPairs + minimumDistance).
  // Returns nullptr when no collision pairs are configured.
  static std::unique_ptr<CollisionMonitor> loadFromTaskFile(
      const std::string& taskFile, const ocs2::PinocchioInterface& pinocchioInterface);

  // Minimum signed distance over all pairs (>0 clear, <0 penetrating).
  ocs2::scalar_t minDistance(const ocs2::vector_t& state) const;

  // true when any pair is closer than minimumDistance.
  bool isInCollision(const ocs2::vector_t& state) const {
    return minDistance(state) < minimumDistance_;
  }

  ocs2::scalar_t getMinimumDistance() const { return minimumDistance_; }
  size_t numCollisionPairs() const { return geometryInterface_.getNumCollisionPairs(); }

 private:
  // mutable: forwardKinematics writes into the PinocchioInterface's data.
  mutable ocs2::PinocchioInterface pinocchioInterface_;
  ocs2::PinocchioGeometryInterface geometryInterface_;
  ocs2::scalar_t minimumDistance_;
};

}  // namespace hyumm_bridge
