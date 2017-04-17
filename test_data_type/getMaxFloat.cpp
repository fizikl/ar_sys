#include <limits>
#include <iostream>
#include <opencv2/opencv.hpp>

#ifdef USE_OMP
#include <omp.h>
#else
int omp_get_max_threads();
int omp_get_thread_num();
#endif

using namespace std;
using namespace cv;
int main(){
	cout << "max float is: " << std::numeric_limits<float>::max() << "\n";
	cout << "min float is: " << std::numeric_limits<float>::min() << "\n";
	cout << "infinity is: " << std::numeric_limits<float>::infinity() << "\n";

	Point2f p1 = Point2f(20,20);
	Point2f p2 = Point2f(30,30);
	Point2f p3 = Point2f(40,40);
	Point2f p4 = Point2f(50,50);
	//std::vector<cv::Point2f> vec = 
}