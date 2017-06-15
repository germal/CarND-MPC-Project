/*                                                                         80->|
 * MPC.h
 *
 * Modifications: James William Dunn
 *          Date: June 15, 2017
 */

#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "timer.h"

using namespace std;

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd, Eigen::VectorXd, vector<double> &, vector<double> &);
};

#endif /* MPC_H */
