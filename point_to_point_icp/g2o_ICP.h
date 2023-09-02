#ifndef G2O_ICP_H
#define G2O_ICP_H
#include <iostream>
#include <random>

#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/solver.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/stuff/sampler.h"
#include "g2o/types/icp/types_icp.h"

void g2o_ICP(std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> &pointPairList, Eigen::Isometry3d &Tx_V1_V2, int optimize_times);

#endif