#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>
#include <Eigen/Core>

int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <input_file>" << std::endl;
        return 1;
    }

    const std::string inputFile = argv[1];
    
    std::ifstream infile(inputFile); // Replace "data.txt" with your actual file path
    if (!infile.is_open()) {
        std::cerr << "Failed to open file." << std::endl;
        return 1;
    }

    std::vector<std::vector<Eigen::Vector3d>> data;

    std::string line;
    while (std::getline(infile, line)) {
        std::istringstream iss(line);
        int timestamp, numPoints;
        if (!(iss >> timestamp >> numPoints)) {
            std::cerr << "Error reading timestamp and number of points." << std::endl;
            return 1;
        }
        //iss >> timestamp >> numPoints;
        
        

        std::vector<Eigen::Vector3d> points;
        for (int i = 0; i < numPoints; ++i) {
            double x, y, z;
            if (!(infile >> x >> y >> z)) {
                std::cerr << "Error reading point data." << std::endl;
                return 1;
            }
            points.push_back(Eigen::Vector3d(x, y, z));
        }
        data.push_back(points);
        char c;
        infile >> c;
    }

    infile.close();

    // Now 'data' contains the vectors of Eigen::Vector3d points for each timestamp
    for (const auto& timestampData : data) {
        for (const Eigen::Vector3d& point : timestampData) {
            std::cout << "x: " << point.x() << " y: " << point.y() << " z: " << point.z() << std::endl;
        }
        std::cout << "------------" << std::endl;
    }

    return 0;
}
