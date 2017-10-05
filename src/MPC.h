#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class MPC {
 public:
  MPC();

  static constexpr double DEGREE_LIMIT = 0.436332;

  virtual ~MPC();


   double a_;
   double steering_delta_;


   std::vector<double> pred_waypoint_x_;
    std::vector<double> pred_waypoint_y_;

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);


};

#endif /* MPC_H */
