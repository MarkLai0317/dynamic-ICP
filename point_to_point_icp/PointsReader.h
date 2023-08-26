#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <Eigen/Core>
#include <sstream>
#include "g2o/stuff/sampler.h"

using namespace Eigen;
using namespace std;

class PointsReader{
public:
    // PointsReader(std::string inputFile): infile(inputFile){
    //     if (!infile.is_open()) {
    //         std::cerr << "Failed to open file." << std::endl;
    //     }
    // }

    // bool readFramePoints(std::vector<Eigen::Vector3d> &points){
        
    //     // read the first line containing "{timestamp} {number of points}"
    //     std::string line;
    //     if(!std::getline(infile, line)){
    //         return false;
    //     }
    //     std::istringstream iss(line);
    //     int timestamp, numPoints;
    //     if (!(iss >> timestamp >> numPoints)) {
    //         std::cerr << "Error reading timestamp and number of points." << std::endl;
    //         return false;
    //     }

    //     // create fixd length vactor of Vector3d
    //     points = std::vector<Eigen::Vector3d>(numPoints);
    //     for(int i = 0; i < numPoints; ++i){
    //         double x, y, z;
    //         if (!(infile >> x >> y >> z)) {
    //             std::cerr << "Error reading point data." << std::endl;
    //         }
    //         points[i] = Eigen::Vector3d(x, y, z);
    //     }

    //     // catch trash charactor
    //     char c;
    //     infile >> c;

    //     return true;


    // }

    PointsReader(std::string inputFile): infile(inputFile), cam(2){
        for (size_t i = 0; i < numberOfPoints; ++i) {
            true_points.push_back(
                Vector3d((g2o::Sampler::uniformRand(0., 1.) - 0.5) * 3,
                        g2o::Sampler::uniformRand(0., 1.) - 0.5,
                        g2o::Sampler::uniformRand(0., 1.) + 10));
        }

        for (size_t i = 0; i < 2; ++i) {
            // set up rotation and translation for this node
            Vector3d t(0, 0, i);
            Quaterniond q;
            q.setIdentity();

            // camera pose
            cam[i] = q;
            cam[i].translation() = t;
        }
    }

    bool readFramePoints(std::vector<Eigen::Vector3d> &points){
        
        if(frame == 2){
            return false;
        }
        points = vector<Vector3d>(numberOfPoints);
        for (size_t i = 0; i < true_points.size(); ++i) {
            
            Vector3d pt0;
            double euc_noise = 0.01;
            pt0 = cam[frame].inverse() * true_points[i];
            pt0 += Vector3d(g2o::Sampler::gaussRand(0., euc_noise),
                    g2o::Sampler::gaussRand(0., euc_noise),
                    g2o::Sampler::gaussRand(0., euc_noise));
            points[i] = pt0;

        }
        frame += 1;

        return true;

    }



    void printVector(std::vector<Eigen::Vector3d> &points){
        for (const Eigen::Vector3d& point : points) {
            std::cout << "x: " << point.x() << " y: " << point.y() << " z: " << point.z() << std::endl;
        }
        std::cout << "------------" << std::endl;
    }


private:
    std::vector<Eigen::Vector3d> true_points;
    vector<Eigen::Isometry3d> cam;
    int frame = 0;
    int numberOfPoints = 1000;

    std::ifstream infile;
    
};