#pragma once

#include "function.h"

void RRT(node(&v), path(&pos), const double(&start)[2], const double(&end)[2], const u_int(&& r_num), const u_int(&& m_index), const cv::Mat (&param_img))
{
	std::random_device rd;
	std::mt19937 mesenne(rd());
	const u_int m = m_index;

	for (u_int i = 0; i < r_num; ++i)
	{
		std::uniform_real_distribution<double> distributionX(START[m][0], END[m][0]);
		std::uniform_real_distribution<double> distributionY(START[m][1], END[m][1]);

		const double x_rand[2] = { distributionX(mesenne), distributionY(mesenne) };

		double min_dist = INT_MAX;
		u_int near_iter = 0;
		double min_costA = INT_MAX;
		u_int cost_iter = 0;

		const u_int N = static_cast<u_int>(v.size());

		for (u_int j = 0; j < N; ++j)
		{
			const double x_near[2] = { v.x[j], v.y[j] };
			const double dist = getDistance(x_rand, x_near);

			if (min_dist > dist)
			{
				min_dist = dist;
				near_iter = j;
			}
		}

		const double x_near[2] = { v.x[near_iter], v.y[near_iter] };
		const double near_rand[2] = { x_rand[0] - x_near[0], x_rand[1] - x_near[1] };
		const double normlized[2] = { near_rand[0] / getDistance(x_rand, x_near) * stepsize, near_rand[1] / getDistance(x_rand, x_near) * stepsize };
		const double x_new[2] = { x_near[0] + normlized[0], x_near[1] + normlized[1] };
		const u_int CheckInBoundNew = check_in_boundary(x_new, gray_img, 1);
		const u_int LineCheck1 = line_check(x_new, x_near, 4, gray_img);

		if (CheckInBoundNew == 1 || LineCheck1 == 1) continue;

		/////////////* choose parent*/////////////

		const vector_ui neighbor_index = find_near_neighbor(v, x_new, 20);
		std::vector<std::tuple<double, double, u_int>> cost_dist;
		std::vector<std::tuple<double, u_int>> cost_angle;

		const u_int index_num = static_cast<u_int>(neighbor_index.size());

		if (index_num)
		{
			for (const auto& k : neighbor_index)
			{
				const double cur_cost[2] = { v.x[k], v.y[k] };
				const double pre_cost[2] = { v.x[v.indPre[k]], v.y[v.indPre[k]] };
				const auto cost = make_cost(pre_cost, cur_cost, x_new);
				cost_dist.emplace_back(std::make_tuple(std::get<0>(cost), std::get<1>(cost), k));
			}

			const u_int costD_num = static_cast<u_int>(cost_dist.size());
			double total_cost_dist = 0.0;

			for (u_int t = 0; t < costD_num; ++t) total_cost_dist += std::get<1>(cost_dist[t]);

			const double avg_cost_dist = total_cost_dist / costD_num;

			for (u_int v = 0; v < costD_num; ++v)
			{
				if (std::get<1>(cost_dist[v]) < avg_cost_dist) cost_angle.emplace_back(std::make_tuple(std::get<0>(cost_dist[v]), std::get<2>(cost_dist[v])));
			}

			const u_int costA_num = static_cast<u_int>(cost_angle.size());

			for (u_int o = 0; o < costA_num; ++o)
			{
				if (min_costA > std::get<0>(cost_angle[o]))
				{
					min_costA = std::get<0>(cost_angle[o]);
					cost_iter = std::get<1>(cost_angle[o]);
				}
			}
		}

		const double x_cost[2] = { v.x[cost_iter], v.y[cost_iter] };
		const u_int LineCheck2 = line_check(x_new, x_cost, 4, gray_img);

		if (LineCheck2 == 1) continue;

		v.setXY(x_new[0], x_new[1], cost_iter);
		const u_int new_iter = static_cast<u_int>(v.size()) - 1;
		cv::line(param_img, cv::Point(x_cost[0], x_cost[1]), cv::Point(x_new[0], x_new[1]), cv::Scalar(255, 0, 0), 1, 4, 0);

		if (getDistance(x_new, end) < 1) break;

		//////////////////////////////////////////
		////////////////* rewire*/////////////////
		vector_ui new_child_index_list;

		if (index_num)
		{
			for (const auto& k : neighbor_index)
			{
				if (k != cost_iter and k != 0)
				{
					const double x_child[2] = { v.x[k], v.y[k] };
					const double x_parent[2] = { v.x[v.indPre[k]], v.y[v.indPre[k]] };
					const double x_grand_parent[2] = { v.x[v.indPre[v.indPre[k]]], v.y[v.indPre[v.indPre[k]]] };

					const auto new_cost = make_cost(x_cost, x_new, x_child);
					const auto neighbor_cost = make_cost(x_grand_parent, x_parent, x_child);

					if (std::get<0>(new_cost) < std::get<0>(neighbor_cost) && std::get<1>(new_cost) < std::get<1>(neighbor_cost))
					{
						const u_int LineCheck3 = line_check(x_child, x_new, 4, gray_img);
						if (LineCheck3 == 1) continue;

						new_child_index_list.emplace_back(k);
					}
				}
			}
		}

		if (new_child_index_list.size())
		{
			for (const auto& k : new_child_index_list)
			{
				const u_int pre_iter = v.indPre[k];
				const double new_child[2] = { v.x[k], v.y[k] };
				v.indPre[k] = new_iter;

				const u_int Bermuda = bermuda(v.indPre, k);

				if (Bermuda == 1)
				{
					v.indPre[k] = pre_iter;
					continue;
				}

				cv::line(param_img, cv::Point(x_new[0], x_new[1]), cv::Point(new_child[0], new_child[1]), cv::Scalar(255, 0, 0), 1, 8, 0);
			}
		}
	}

	if (true)
	{
		pos.setXY(end[0], end[1]);

		pos.setXY(v.x.back(), v.y.back());

		u_int path_index = v.indPre.back();

		while (true)
		{
			pos.setXY(v.x[path_index], v.y[path_index]);
			path_index = v.indPre[path_index];

			if (path_index == 0) break;
		}

		pos.setXY(start[0], start[1]);
		const u_int Range_Num = static_cast<u_int>(pos.size());

		for (u_int j = 1; j < Range_Num; ++j)
		{
			cv::line(img, cv::Point(pos.x[j], pos.y[j]), cv::Point(pos.x[j - 1], pos.y[j - 1]), cv::Scalar(0, 0, 255), 1, 8, 0);
		}
	}
	else std::cout << "no path to end" << std::endl;
}