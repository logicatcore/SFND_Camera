#include <iostream>
#include <numeric>
#include <unordered_set>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "structIO.hpp"

using namespace std;

std::unordered_set<int> RansacPlaneIdx(const std::vector<LidarPoint> &cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	int idx1{0}, idx2{0}, idx3{0};
	long unsigned int maxIdx{cloud.size()};
	float A{0}, B{0}, C{0}, D{0}, denom{0};
	float x1, x2, x3, y1, y2, y3, z1, z2, z3;

	// For max iterations 
	while (maxIterations--) {
		std::unordered_set<int> inliers;
		// Randomly sample subset and fit plane
		idx1 = std::rand() % maxIdx;
		do {
			idx2 = std::rand() % maxIdx;
		}while(idx2 == idx1);
		do {
			idx3 = std::rand() % maxIdx;
		}while(idx3 == idx1 | idx3 == idx2);
		
		x1 = cloud[idx1].x;
		y1 = cloud[idx1].y;
		z1 = cloud[idx1].z;

		x2 = cloud[idx2].x;
		y2 = cloud[idx2].y;
		z2 = cloud[idx2].z;

		x3 = cloud[idx3].x;
		y3 = cloud[idx3].y;
		z3 = cloud[idx3].z;

		A = (y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1); 
		B = (z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1);
		C = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);
		D = - (A * x1 + B * y1 + C * z1);

		denom = sqrt(pow(A, 2) + pow(B, 2) + pow(C, 2));
		// Measure distance between every point and fitted line
		// If distance is smaller than threshold count it as inlier
		for (int idx = 0; idx < cloud.size(); idx++){
				if (abs(A * cloud[idx].x + B * cloud[idx].y + C * cloud[idx].z + D)/denom < distanceTol) {
					inliers.insert(idx);
				}
		}
		if (inliersResult.size() < inliers.size()){
				inliersResult = inliers;
		}
	}
	// Return indicies of inliers from fitted line with most inliers
	return inliersResult;
}

void showLidarTopview()
{
    std::vector<LidarPoint> lidarPoints;
    readLidarPts("../dat/C51_LidarPts_0000.dat", lidarPoints);

    cv::Size worldSize(10.0, 20.0); // width and height of sensor field in m
    cv::Size imageSize(1000, 2000); // corresponding top view image in pixel

    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(0, 0, 0));
    
    // ideally we should be finding the max along the x direction and use it to assign a scaled color to each point
    // auto maxXPt = std::max_element(lidarPoints.begin(), lidarPoints.end(), [](LidarPoint &pt1, LidarPoint &pt2){return pt1.x > pt1.x;});    
        
    // ransac results are not consistent enough
    // std::unordered_set<int> inliersIndices = RansacPlaneIdx(lidarPoints, 50, 1.2);
    
    // plot Lidar points into image
    for (auto it = lidarPoints.begin(); it != lidarPoints.end(); ++it)
    {
        //  since z = 0 is the roof of the car, the points to be discarded i.e. ground points take a negative value
        if ((*it).z > -1.4)
        {
            float xw = (*it).x; // world position in m with x facing forward from sensor
            float yw = (*it).y; // world position in m with y facing left from sensor

            int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
            int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

            // TODO: 
            // 1. Change the color of the Lidar points such that 
            // X=0.0m corresponds to red while X=20.0m is shown as green.
            // 2. Remove all Lidar points on the road surface while preserving 
            // measurements on the obstacles in the scene.
            int g = min(255, (int)(255 * abs(xw/20)));
            int r = min(255, (int)(255 * (1 - abs(xw/20))));
            cv::circle(topviewImg, cv::Point(x, y), 5, cv::Scalar(0, g, r), -1);
        }
    }

    // plot distance markers
    float lineSpacing = 2.0; // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i)
    {
        int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }

    // display image
    string windowName = "Top-View Perspective of LiDAR data";
    cv::namedWindow(windowName, 2);
    cv::imshow(windowName, topviewImg);
    cv::waitKey(0); // wait for key to be pressed
}

int main()
{
    showLidarTopview();
}