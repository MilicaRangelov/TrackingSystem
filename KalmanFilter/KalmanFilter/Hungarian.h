#pragma once
#include <iostream>
#include <vector> 
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

struct Position {
	int i;
	int j;
};

class Hungarian
{
private:
	MatrixXd assignments_copy;

	vector<vector<bool>> zero;
	vector<vector<bool>> zero_copy;

	vector<Position> marked_zero;
	vector<int> marked_rows;
	vector<int> marked_cols;

	vector<int> non_marked_rows;
	vector<int> non_marked_cols;

	int num_zero;

	//initialization 
	void initial_non_marked_cols();
	void initial_non_marked_rows();
	void copy_boolean_matrix();
	void clear_vectors();

	void step2();

	// step 2-1
	void min_zero_row();
	void find_position(int min,int imin);

	//step 2-2
	void mark_matrix();
	void compare_column_index(const int& column);

	//step 3
	void identify_the_result();

	//step 4
	void adjust_matrix();

	//step 5
	void calculate_the_answer();

public:
	MatrixXd assignments;

	Hungarian(const MatrixXd& assignments);
	~Hungarian();

	// substract lowest value in each row  and each column
	void substract_rows_columns();
	vector<Position> positions();

};

