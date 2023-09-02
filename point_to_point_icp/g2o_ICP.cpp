#include "g2o_ICP.h"
using namespace Eigen;
using namespace std;
using namespace g2o;
void g2o_ICP(std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> &pointPairList, Eigen::Isometry3d &Tx_V1_V2, int optimize_times){
    
    double euc_noise = 0.01; // noise in position, m
        //  double outlier_ratio = 0.1;

        SparseOptimizer optimizer;
        optimizer.setVerbose(false);

        // variable-size block solver
        g2o::OptimizationAlgorithmLevenberg *solver =
            new g2o::OptimizationAlgorithmLevenberg(make_unique<BlockSolverX>(
                make_unique<LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>>()));

        optimizer.setAlgorithm(solver);


        int vertex_id = 0;
        for (size_t i = 0; i < 2; ++i)
        {
            // set up rotation and translation for this node
            Vector3d t(0, 0, i);
            Quaterniond q;
            q.setIdentity();

            Eigen::Isometry3d cam; // camera pose
            cam = q;
            cam.translation() = t;

            // set up node
            VertexSE3 *vc = new VertexSE3();
            vc->setEstimate(cam);

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
            Vector3d pt0, pt1;
            // pt0 = vp0->estimate().inverse() * true_points[i];
            // pt1 = vp1->estimate().inverse() * true_points[i];

        

            // add in noise
            // pt0 += Vector3d(g2o::Sampler::gaussRand(0., euc_noise),
            //                 g2o::Sampler::gaussRand(0., euc_noise),
            //                 g2o::Sampler::gaussRand(0., euc_noise));

            // pt1 += Vector3d(g2o::Sampler::gaussRand(0., euc_noise),
            //                 g2o::Sampler::gaussRand(0., euc_noise),
            //                 g2o::Sampler::gaussRand(0., euc_noise));

            pt0 = pointPairList[i].first;
            pt1 = pointPairList[i].second;

            // form edge, with normals in varioius positions
            Vector3d nm0, nm1;
            nm0 << 0, i, 1;
            nm1 << 0, i, 1;
            nm0.normalize();
            nm1.normalize();

            Edge_V_V_GICP *e // new edge with correct cohort for caching
                = new Edge_V_V_GICP();

            e->setVertex(0, vp0); // first viewpoint
            e->setVertex(1, vp1); // second viewpoint

            EdgeGICP meas;
            meas.pos0 = pt0;
            meas.pos1 = pt1;

            // cout << "point1:\n"<<pt0 << "pooint2:\n" << pt1 << endl;
            // meas.normal0 = nm0;
            // meas.normal1 = nm1;

            e->setMeasurement(meas);
            //        e->inverseMeasurement().pos() = -kp;

            meas = e->measurement();
            // use this for point-plane
            // e->information() = meas.prec0(0.01);

            // use this for point-point
            e->information().setIdentity();

            // e->setRobustKernel(true);
            // e->setHuberWidth(0.01);

            optimizer.addEdge(e);
        }

        // move second cam off of its true position
        VertexSE3 *vc =
            dynamic_cast<VertexSE3 *>(optimizer.vertices().find(1)->second);
        Eigen::Isometry3d cam = vc->estimate();
        cam.translation() = Vector3d(0, 0, 0.2);
        //vc->setEstimate(cam);
        vc->setEstimate(Tx_V1_V2);

        optimizer.initializeOptimization();
        optimizer.computeActiveErrors();
        cout << "Initial chi2 = " << FIXED(optimizer.chi2()) << endl;

        optimizer.setVerbose(true);

        optimizer.optimize(optimize_times);

        cout << endl
            << "Second vertex result" << endl;
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

        // update new Tx_V1_V2 and points2

        Tx_V1_V2 = vc->estimate();
}
