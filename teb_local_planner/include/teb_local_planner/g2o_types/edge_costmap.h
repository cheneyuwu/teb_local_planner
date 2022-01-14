#ifndef EDGE_COSTMAP_H_
#define EDGE_COSTMAP_H_

#include "teb_local_planner/costmap.h"
#include "teb_local_planner/obstacles.h"
#include "teb_local_planner/robot_footprint_model.h"
#include "teb_local_planner/g2o_types/vertex_pose.h"
#include "teb_local_planner/g2o_types/base_teb_edges.h"
#include "teb_local_planner/g2o_types/penalties.h"
#include "teb_local_planner/teb_config.h"


namespace teb_local_planner
{

class EdgeCostMap : public BaseTebUnaryEdge<1, const CostMap*, VertexPose>
{
public:

  /**
   * @brief Construct edge.
   */
  EdgeCostMap()
  {
    _measurement = nullptr;
  }

  /**
   * @brief Actual cost function
   */
  void computeError()
  {
    TEB_ASSERT_MSG(cfg_ && _measurement && robot_model_, "You must call setTebConfig(), setCostmap() and setRobotModel() on EdgeCostMap()");
    const VertexPose* bandpt = static_cast<const VertexPose*>(_vertices[0]);
    float interpolation = 0.0;
    _measurement->getValue(bandpt->pose(), interpolation);
    _error[0] = (double)interpolation;
    TEB_ASSERT_MSG(std::isfinite(_error[0]), "EdgeCostMap::computeError() _error[0]=%f\n",_error[0]);
  }

  /**
   * @brief Set pointer to associated predicted costmap for the underlying cost function
   * @param costmap 2D costmap containing information about collision probabilities
   */
  void setCostMap(const CostMap* costmap)
  {
    _measurement = costmap;
  }

  /**
   * @brief Set pointer to the robot model
   * @param robot_model Robot model required for distance calculation
   */
  void setRobotModel(const BaseRobotFootprintModel* robot_model)
  {
    robot_model_ = robot_model;
  }

  /**
   * @brief Set all parameters at once
   * @param cfg TebConfig class
   * @param robot_model Robot model required for distance calculation
   * @param costmap 2D costmap containing information about collision probabilities
   */
  void setParameters(const TebConfig& cfg, const BaseRobotFootprintModel* robot_model, CostMap* costmap)
  {
    cfg_ = &cfg;
    robot_model_ = robot_model;
    _measurement = costmap;
  }


protected:
  const BaseRobotFootprintModel* robot_model_; //!< Store pointer to robot_model

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

} // end namespace
#endif
