#include "read_points.h"
#include <vector>
#include <Eigen/Core>

int main(int argc, char* argv[]){
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <input_file>" << std::endl;
        return 1;
    }

    const std::string inputFile = argv[1];

    PointsReader pointsReader(inputFile);
    
    std::vector<Eigen::Vector3d> points;
    while(pointsReader.readFramePoints(points)){
        pointsReader.printVector(points);
    }

    return 0;
}