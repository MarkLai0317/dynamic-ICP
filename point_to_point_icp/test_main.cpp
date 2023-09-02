// g2o - General Graph Optimization
// Copyright (C) 2011 Kurt Konolige
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <stdint.h>

#include <iostream>
#include <random>

#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/solver.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/stuff/sampler.h"
#include "g2o/types/icp/types_icp.h"

#include "knn.h"
#include "PointsReader.h"
#include "g2o_ICP.h"

using namespace Eigen;
using namespace std;
using namespace g2o;



void transformPoints(vector<Vector3d> &points, Isometry3d transformation){
    for(int i = 0; i < points.size(); i++){
        points[i] = transformation * points[i];
    }
    return;
}


int main(int argc, char* argv[])
{
   
    if (argc != 4) {
        std::cerr << "Usage: " << argv[0] << " <icp_iterate_times>" << " <optimize_times>" << "<file_path>"<<std::endl;
        return 1;
    }

    int icp_iterate_times = std::stoi(argv[1]);
    int optimize_times = std::stoi(argv[2]);
    string file_path = argv[3];
    
    PointsReader pointsReader(file_path);

    // read first frame data
    std::vector<Eigen::Vector3d> points1;
    pointsReader.readFramePoints(points1);
    std::vector<Eigen::Vector3d> points2;
    pointsReader.readFramePoints(points2);

    //pointsReader.printVector(point);

    // vector<Vector3d> true_points;
    // for (size_t i = 0; i < 1000; ++i)
    // {
    //     true_points.push_back(
    //         Vector3d((g2o::Sampler::uniformRand(0., 1.) - 0.5) * 3,
    //                  g2o::Sampler::uniformRand(0., 1.) - 0.5,
    //                  g2o::Sampler::uniformRand(0., 1.) + 10));
    // }

    // set up two poses

    // initial Tx_V1_V2
    Eigen::Isometry3d Tx_V1_V2;
    Quaterniond q;
    q.setIdentity();
    Tx_V1_V2 = q;
    Tx_V1_V2.translation() = Vector3d(0, 0, 0);

    vector<Isometry3d>transformationList;
    for(int i = 0; i < icp_iterate_times; i++){
        
        cout << "\n\niteration number: " << i << endl << endl;
        
        std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> pointPairList;
        double threshold = std::numeric_limits<double>::max();


        knn(pointPairList, points1, points2, 1000000);
        g2o_ICP(pointPairList, Tx_V1_V2, optimize_times);

        
        transformationList.push_back(Tx_V1_V2);
        transformPoints(points2, Tx_V1_V2);
        cout << endl;
    }

    // merge transformation;
    Vector3d t(0, 0, 0);
    Isometry3d final_Tx_V1_V2;
   
    final_Tx_V1_V2 = q;
    final_Tx_V1_V2.translation() = t;

    for(int i = 0; i < transformationList.size(); ++i){
        final_Tx_V1_V2 = transformationList[i] * final_Tx_V1_V2; 
    }


    cout << endl
        << "final second vertex should be near 0,0,1" << endl;
    cout << final_Tx_V1_V2.translation().transpose()
        << endl;
        


    
}