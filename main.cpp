#include "RRT.h"

int main()
{
	v1.setXY(robot[0], robot[1], 0);
	v2.setXY(waypoint1[0], waypoint1[1], 0);
	v3.setXY(waypoint1[0], waypoint2[1], 0);

	img.copyTo(gray_img);
	cv::circle(img, cv::Point(robot[0], robot[1]), 1, cv::Scalar(0, 255, 0), 5, 8);
	cv::circle(img, cv::Point(waypoint1[0], waypoint1[1]), 1, cv::Scalar(255, 255, 0), 5, 8);
	cv::circle(img, cv::Point(waypoint2[0], waypoint2[1]), 1, cv::Scalar(255, 255, 0), 5, 8);
	cv::circle(img, cv::Point(goal[0], goal[1]), 1, cv::Scalar(0, 255, 0), 5, 8);
	
	
	RRT(v1, pos1, robot, waypoint1, 3001, 0, img);
	RRT(v2, pos2, waypoint1, waypoint2, 4001, 1, img);
	RRT(v3, pos3, waypoint2, goal, 5001, 2, img);
	
	path Final_1 = PathRefine(pos1, 1);
	path Final_2 = PathRefine(pos2, 1);
	Final_2.popXY();
	path Final_3 = PathRefine(pos3, -1);
	Final_3.popXY();

	Final_3.insert_custom(Final_2);
	Final_3.insert_custom(Final_1);
	
	const u_int final_length = static_cast<u_int>(Final_3.size());
	
	for (u_int i = 0; i < final_length - 1; ++i)
	{
		cv::line(img, cv::Point(Final_3.x[i], Final_3.y[i]), cv::Point(Final_3.x[i+1], Final_3.y[i+1]), cv::Scalar(0, 0, 0), 1, 8, 0);
	}
	
	cv::imshow("Local map with an obstacle and line constraints", img);
	cv::imwrite("output.jpg", img);
	cv::waitKey(0);
	return 0;
}