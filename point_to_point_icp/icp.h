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

#ifndef ICP_H
#define ICP_H
#include <stdint.h>

#include <iostream>
#include <random>
#include <utility>
#include <vector>
#include <Eigen/Core>

#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/solver.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/stuff/sampler.h"
#include "g2o/types/icp/types_icp.h"

using namespace Eigen;
using namespace std;
using namespace g2o;

void icp(vector<pair<Vector3d, Vector3d>> &pointPairList, Isometry3d &Tx_V1_V2, int optimizeTimes)
{
    cout << "pointPairList length: "<< pointPairList.size() << endl;
    double euc_noise = 0.01; // noise in position, m
    //  double outlier_ratio = 0.1;

    SparseOptimizer optimizer;
    optimizer.setVerbose(false);

    // variable-size block solver
    g2o::OptimizationAlgorithmLevenberg *solver =
        new g2o::OptimizationAlgorithmLevenberg(make_unique<BlockSolverX>(
            make_unique<LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>>()));

    optimizer.setAlgorithm(solver);

    // vector<Vector3d> true_points;
    // for (size_t i = 0; i < 1000; ++i)
    // {
    //     true_points.push_back(
    //         Vector3d((g2o::Sampler::uniformRand(0., 1.) - 0.5) * 3,
    //                  g2o::Sampler::uniformRand(0., 1.) - 0.5,
    //                  g2o::Sampler::uniformRand(0., 1.) + 10));
    // }

    // set up two poses
    int vertex_id = 0;
    for (size_t i = 0; i < 2; ++i)
    {
        // set up rotation and translation for this node
        Vector3d t(0, 0, 0);
        Quaterniond q;
        q.setIdentity();

        Eigen::Isometry3d cam; // camera pose
        cam = q;
        cam.translation() = t;

        // set up node
        VertexSE3 *vc = new VertexSE3();
        if(i == 1){
            vc->setEstimate(Tx_V1_V2);
        }
        else{
            vc->setEstimate(cam);
        }
        

        vc->setId(vertex_id); // vertex id

        cerr << t.transpose() << " | " << q.coeffs().transpose() << endl;

        // set first cam pose fixed
        if (i == 0)
            vc->setFixed(true);

        // add to optimizer
        optimizer.addVertex(vc);

        vertex_id++;
    }

    // set up point matches
    for (size_t i = 0; i < pointPairList.size(); ++i)
    {
        // get two poses
        VertexSE3 *vp0 =
            dynamic_cast<VertexSE3 *>(optimizer.vertices().find(0)->second);
        VertexSE3 *vp1 =
            dynamic_cast<VertexSE3 *>(optimizer.vertices().find(1)->second);

        // calculate the relative 3D position of the point
        //Vector3d pt0, pt1;
       

        // add in noise
        // pt0 += Vector3d(g2o::Sampler::gaussRand(0., euc_noise),
        //                 g2o::Sampler::gaussRand(0., euc_noise),
        //                 g2o::Sampler::gaussRand(0., euc_noise));

        // pt1 += Vector3d(g2o::Sampler::gaussRand(0., euc_noise),
        //                 g2o::Sampler::gaussRand(0., euc_noise),
        //                 g2o::Sampler::gaussRand(0., euc_noise));

        // form edge, with normals in varioius positions
        // Vector3d nm0, nm1;
        // nm0 << 0, i, 1;
        // nm1 << 0, i, 1;
        // nm0.normalize();
        // nm1.normalize();

        Edge_V_V_GICP *e // new edge with correct cohort for caching
            = new Edge_V_V_GICP();

        e->setVertex(0, vp0); // first viewpoint
        e->setVertex(1, vp1); // second viewpoint

        EdgeGICP meas;
        meas.pos0 = pointPairList[i].first;
        meas.pos1 = pointPairList[i].second;
        
        //cout << "point1:\n"<<pointPairList[i].first << "\npooint2:\n" << pointPairList[i].second << endl; 
        // meas.normal0 = nm0;
        // meas.normal1 = nm1;

        e->setMeasurement(meas);
        //        e->inverseMeasurement().pos() = -kp;

        meas = e->measurement();
        // // use this for point-plane
        e->information() = meas.prec0(0.01);

        // use this for point-point
        // e->information().setIdentity();

        //    e->setRobustKernel(true);
        // e->setHuberWidth(0.01);

        optimizer.addEdge(e);
    }

    // move second cam off of its true position
    // VertexSE3 *vc =
    //     dynamic_cast<VertexSE3 *>(optimizer.vertices().find(1)->second);
    // Eigen::Isometry3d cam = vc->estimate();
    // cam.translation() = Vector3d(0, 0, 0.2);
    // vc->setEstimate(cam);

    optimizer.initializeOptimization();
    optimizer.computeActiveErrors();
    cout << "Initial chi2 = " << FIXED(optimizer.chi2()) << endl;

    optimizer.setVerbose(true);

    optimizer.optimize(10);

    cout << endl << endl;
    cout << dynamic_cast<VertexSE3 *>(optimizer.vertices().find(0)->second)
                ->estimate()
                .translation()
                .transpose()
         << endl;
    cout << dynamic_cast<VertexSE3 *>(optimizer.vertices().find(1)->second)
                ->estimate()
                .translation()
                .transpose()
         << endl;

    Tx_V1_V2 = dynamic_cast<VertexSE3 *>(optimizer.vertices().find(1)->second)->estimate();

    
}

#endif