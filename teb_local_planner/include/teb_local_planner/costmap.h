#ifndef COSTMAP_H_
#define COSTMAP_H_

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <Eigen/Geometry>

#include <teb_local_planner/pose_se2.h>
#include <teb_local_planner/g2o_types/vertex_pose.h>

namespace teb_local_planner
{

/**
 * @class CostMap
 * @brief Class that defines the interface to interact with the costmap of the collision checker Neural Network
 */
class CostMap
{
public:
  /** @brief Default constructor of the CostMap class */
  CostMap() = default;
  /** @brief Virtual destructor. */
  virtual ~CostMap() = default;
  /**
   * @brief Interpolate costmap value at pose
   * @param pos (x, y) position where to interpolate from the costmap
   * @param value
   */
  virtual void getValue(const PoseSE2& pose, float &value) const = 0;
  virtual void getValue(const VertexPose& pose, float &value) const = 0;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

//! Abbrev. for shared CostMap pointers
typedef std::shared_ptr<CostMap> CostMapPtr;
//! Abbrev. for shared CostMap const pointers
typedef std::shared_ptr<const CostMap> CostMapConstPtr;
//! Abbrev. for containers storing multiple CostMaps
typedef std::vector<CostMapPtr> CostMapContainer;

} // namespace teb_local_planner
#endif /* COSTMAP_H_ */