#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

#define MPC_SIZE_N			10
#define MPC_DT				0.05
#define MPC_SPEED			50

#define MPC_OUT_SIZE		4
#define MPC_X_OFST			0
#define MPC_Y_OFST			1
#define MPC_SPEED_OFST		2
#define MPC_THROTTLE_OFFSET	3

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */
