#include "hyumm_bridge/CollisionMonitor.h"

#include <algorithm>
#include <limits>

#include <pinocchio/algorithm/kinematics.hpp>

#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/misc/LoadStdVectorOfPair.h>

#include "hyumm_ocs2/HyummPinocchioMapping.h"

namespace hyumm_bridge {

CollisionMonitor::CollisionMonitor(
    const ocs2::PinocchioInterface& pinocchioInterface,
    const std::vector<std::pair<std::string, std::string>>& collisionLinkPairs,
    ocs2::scalar_t minimumDistance)
    : pinocchioInterface_(pinocchioInterface),
      geometryInterface_(pinocchioInterface_, collisionLinkPairs, {}),
      minimumDistance_(minimumDistance) {}

std::unique_ptr<CollisionMonitor> CollisionMonitor::loadFromTaskFile(
    const std::string& taskFile, const ocs2::PinocchioInterface& pinocchioInterface) {
  std::vector<std::pair<std::string, std::string>> linkPairs;
  ocs2::loadData::loadStdVectorOfPair(taskFile, "selfCollision.collisionLinkPairs",
                                      linkPairs, false);
  if (linkPairs.empty()) return nullptr;
  ocs2::scalar_t minDist = 0.02;
  ocs2::loadData::loadCppDataType(taskFile, "selfCollision.minimumDistance", minDist);
  return std::make_unique<CollisionMonitor>(pinocchioInterface, linkPairs, minDist);
}

ocs2::scalar_t CollisionMonitor::minDistance(const ocs2::vector_t& state) const {
  // OCS2 state -> Pinocchio config (identity for the holonomic base mapping).
  static const hyumm_ocs2::HyummPinocchioMapping<ocs2::scalar_t> mapping;
  const ocs2::vector_t q = mapping.getPinocchioJointPosition(state);

  const auto& model = pinocchioInterface_.getModel();
  auto& data = pinocchioInterface_.getData();
  pinocchio::forwardKinematics(model, data, q);

  const auto results = geometryInterface_.computeDistances(pinocchioInterface_);
  ocs2::scalar_t md = std::numeric_limits<ocs2::scalar_t>::max();
  for (const auto& r : results) {
    md = std::min(md, static_cast<ocs2::scalar_t>(r.min_distance));
  }
  return md;
}

}  // namespace hyumm_bridge
