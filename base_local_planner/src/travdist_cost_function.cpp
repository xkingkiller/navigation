#include <base_local_planner/travdist_cost_function.h>

#include <cmath>

namespace base_local_planner {

TravDistCostFunction::TravDistCostFunction() {
}

TravDistCostFunction::~TravDistCostFunction() {
}

double TravDistCostFunction::scoreTrajectory(Trajectory &traj) {
	int n = traj.getPointsSize();
	if (n <= 0)
		return 1.0;
	double x0, y0, theta0, xn, yn, thetan;
	traj.getPoint(0, x0, y0, theta0);
	traj.getPoint(n - 1, xn, yn, thetan);
	double x_diff = xn - x0;
	double y_diff = yn - y0;
	double dist = sqrt(x_diff * x_diff + y_diff * y_diff);
	if(dist < 0.1)
		return 1.0;
	else
		return (0.1 / dist);

	return 1.0;
}

} /* namespace base_local_planner */
