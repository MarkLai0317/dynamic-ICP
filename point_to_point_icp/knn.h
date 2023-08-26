#ifndef KNN_H
#define KNN_H

#include <Eigen/Core>
#include <vector>
#include <utility>

using namespace std;
using namespace Eigen;



// find point1 that is pair with point2(以point2 為主)
void knn(vector<pair<Vector3d, Vector3d>> &pointPairList , vector<Vector3d> &points1, vector<Vector3d> &points2, double threshold = 0){
    
    
    for(int i = 0; i < points2.size();++i){
        double minDistance = threshold;
        bool smallerThanThreshold = false;
        Vector3d firstPoint;

        for(int j = 0; j < points1.size();++j){
            double distance = (points2[i] - points1[j]).norm();
            if(distance < threshold &&  distance < minDistance ){
                smallerThanThreshold = true;
                minDistance = distance;
                firstPoint = points1[j];
            }
        }
        if(smallerThanThreshold){
            pair<Vector3d, Vector3d> pointPair(firstPoint, points2[i]);
            pointPairList.push_back(pointPair);
        }
    }

    return;
    // for(int i = 0; i < points2.size();++i){
    //     pair<Vector3d, Vector3d> pointPair(points1[i], points2[i]);
    //     pointPairList.push_back(pointPair);
    // }
    // return;

}


#endif // KNN_H