#pragma once

#include <cmath>
#include <tuple>
#include "set_params.h"
#include <iostream>
#include <random>

const double getDistance(const double(&front)[2], const double(&back)[2])
{
	return sqrt(pow(front[0] - back[0], 2) + pow(front[1] - back[1], 2));
}

const vector_d put_element(const double(&small), const double(&big), const u_int (&limit_range))
{
	vector_d tmp_list;
	
	for (auto i = small - limit_range; i <= big + limit_range; ++i) tmp_list.emplace_back(i);
	return tmp_list;
}

const u_int line_check(const double(&axis1)[2], const double(&axis2)[2], const u_int limit_range, const cv::Mat (&gray_img))
{
	vector_d y_list;
	vector_d x_list;

	if (axis1[1] > axis2[1]) y_list = put_element(axis2[1], axis1[1], limit_range);
	else if (axis1[1] < axis2[1]) y_list = put_element(axis1[1], axis2[1], limit_range);
	else y_list = put_element(axis1[1], axis1[1], limit_range);

	if (axis1[0] > axis2[0]) x_list = put_element(axis2[0], axis1[0], limit_range);
	else if (axis1[0] < axis2[0]) x_list = put_element(axis1[0], axis2[0], limit_range);
	else x_list = put_element(axis1[0], axis1[0], limit_range);

	for (const auto& y : y_list)
	{
		for (const auto& x : x_list)
		{
			uchar pixel = gray_img.at<cv::Vec3b>(static_cast<u_int>(y), static_cast<u_int>(x))[0];
			if (static_cast<u_int>(pixel) <= 205) return 1;
		}
	}

	return 0;
}

const u_int check_in_boundary(const double(&axis)[2], const cv::Mat gray_img, const u_int (&limit_range))
{
	vector_d y_list;
	vector_d x_list;

	y_list = put_element(axis[1], axis[1], limit_range);
	x_list = put_element(axis[0], axis[0], limit_range);

	for (const auto& y : y_list)
	{
		for (const auto& x : x_list)
		{
			uchar pixel = gray_img.at<cv::Vec3b>(static_cast<u_int>(y), static_cast<u_int>(x))[0];
			if (static_cast<u_int>(pixel) <= 205) return 1;
		}
	}

	return 0;
}

const vector_ui find_near_neighbor(node v, const double(&new_axis)[2], const u_int(&set_radius))
{
	const u_int count = static_cast<u_int>(v.size());

	vector_ui index;

	for (u_int i = 0; i < count; ++i)
	{
		const double x_neighbor[2] = { v.x[i], v.y[i] };
		if (getDistance(new_axis, x_neighbor) < set_radius) index.emplace_back(i);
	}

	return index;
}

const auto make_cost(const double(&pre)[2], const double(&cur)[2], const double(&add)[2])
{
	const double cur_angle = std::atan2(add[1] - cur[1], add[0] - cur[0]) * 180.0 / PI;
	const double pre_angle = std::atan2(cur[1] - pre[1], cur[0] - pre[0]) * 180.0 / PI;
	const double diff_angle = abs(pre_angle - cur_angle);
	const double dist = getDistance(add, cur);

	return std::tuple<double, double>(diff_angle, dist);
}

const u_int bermuda(const vector_ui indPre, const u_int(&index))
{
	u_int tmp_index = indPre[index];
	while (true)
	{
		if (tmp_index == index) return 1;
		else tmp_index = indPre[tmp_index];

		if (tmp_index == 0) break;
	}

	return 0;
}

path PathRefine(path(&tmp_path), const u_int(& control))
{
	path refined_path;
	const u_int key = control;

Recursive_Refine:
	const u_int pathLength = static_cast<u_int>(tmp_path.size());
	double min_angle = INT_MAX;
	u_int min_angle_index = 0;

	for (u_int i = 1; i < pathLength; ++i)
	{
		const double x_diff = tmp_path.x[0] - tmp_path.x[i];
		const double y_diff = tmp_path.y[0] - tmp_path.y[i];
		const double angle = abs(std::atan2(y_diff, x_diff) * 180.0 / PI);
		if (key == 1)
		{
			const u_int CheckInBound = check_in_boundary({ tmp_path.x[i], tmp_path.y[i] }, gray_img, 3);
			const u_int LineCheck = line_check({ tmp_path.x[i], tmp_path.y[i] }, { tmp_path.x[0], tmp_path.y[0] }, 0, gray_img);
			if (CheckInBound == 1 || LineCheck == 1) continue;
		}
		else if (key == -1)
		{
			const u_int CheckInBound = check_in_boundary({ tmp_path.x[i], tmp_path.y[i] }, gray_img, 1);
			if (CheckInBound == 1) continue;
		}
		

		if (min_angle > angle)
		{
			min_angle = angle;
			min_angle_index = i;
		}
	}

	refined_path.setXY(tmp_path.x[0], tmp_path.y[0]);

	tmp_path.x.erase(tmp_path.x.begin()+0, tmp_path.x.begin() + min_angle_index);
	tmp_path.y.erase(tmp_path.y.begin()+0, tmp_path.y.begin() + min_angle_index);

	if (static_cast<u_int>(tmp_path.size()) > 1) goto Recursive_Refine;
	
	std::cout << "acquire refined path" << std::endl;
	return refined_path;
}