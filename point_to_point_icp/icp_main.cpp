#include "knn.h"
#include "icp.h"
#include "PointsReader.h"

#include "Eigen/Core"
#include <vector>
#include <utility>
#include <string>
#include <limits>

int main(int argc, char* argv[]){
    if (argc != 4) {
        std::cerr << "Usage: " << argv[0] << " <input_file>" << " <icp_iterate_times>" << " <optimize_times>"<<std::endl;
        return 1;
    }

    const std::string inputFile = argv[1];

    PointsReader pointsReader(inputFile);
    
    // read first frame data
    std::vector<Eigen::Vector3d> points1;
    pointsReader.readFramePoints(points1);
    std::vector<Eigen::Vector3d> points2;

    while(pointsReader.readFramePoints(points2)){
        
        Eigen::Isometry3d Tx_V1_V2;
        Tx_V1_V2.translation() = Vector3d(0, 0, 0.2);
        int icpIterateTimes = std::stoi(argv[2]);

        for(int i = 0; i < icpIterateTimes; ++i){

            std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> pointPairList;
            double threshold = std::numeric_limits<double>::max();
            knn(pointPairList, points1, points2, threshold);
            int optimizeTimes = std::stoi(argv[3]);
            icp(pointPairList, Tx_V1_V2, optimizeTimes);
        }
 
        points1 = points2;
    }

    return 0;
}