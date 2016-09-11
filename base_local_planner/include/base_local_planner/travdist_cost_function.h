#ifndef TRAVDIST_COST_FUNCTION_H_
#define TRAVDIST_COST_FUNCTION_H_

#include <base_local_planner/trajectory_cost_function.h>
#include <Eigen/Core>

namespace base_local_planner {

class TravDistCostFunction: public base_local_planner::TrajectoryCostFunction {
public:
	TravDistCostFunction();
  virtual ~TravDistCostFunction();

  double scoreTrajectory(Trajectory &traj);

  bool prepare() {return true;};

};

} /* namespace base_local_planner */
#endif /* TRAVDIST_COST_FUNCTION_H_ */
