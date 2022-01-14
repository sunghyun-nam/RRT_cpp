#pragma once

#include <vector>

typedef std::vector<double> vector_d;
typedef std::vector<unsigned int> vector_ui;
typedef unsigned int u_int;

class path
{
public:
	vector_d x;
	vector_d y;

	virtual void setXY(const double&t_x, const double&t_y)
	{
		x.emplace_back(t_x);
		y.emplace_back(t_y);
	}
	
	void popXY()
	{
		x.pop_back();
		y.pop_back();
	}

	void insert_custom(path (&other))
	{
		x.insert(x.end(), other.x.begin(), other.x.end());
		y.insert(y.end(), other.y.begin(), other.y.end());
	}

	auto size()
	{
		return x.size();
	}
};

class node : public path
{
public:
	vector_ui indPre;

	double operator [](const u_int index)
	{
		return x[index];
	}

	virtual void setXY(const double&t_x, const double&t_y, const u_int &t_indPre)
	{
		x.emplace_back(t_x);
		y.emplace_back(t_y);
		indPre.emplace_back(t_indPre);
	}
};