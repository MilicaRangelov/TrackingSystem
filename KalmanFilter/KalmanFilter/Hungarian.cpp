#include "Hungarian.h"

void Hungarian::initial_non_marked_cols()
{
	for (int j = 0; j < this->assignments.cols(); j++) {
		this->non_marked_cols.push_back(j);
	}
}

void Hungarian::initial_non_marked_rows()
{
	for (int j = 0; j < this->assignments.rows(); j++) {

		this->non_marked_rows.push_back(j);
	}
}

void Hungarian::copy_boolean_matrix()
{
	for (int i = 0; i < this->zero.size(); i++) {
		for (int j = 0; j < this->zero[i].size(); j++) {

			this->zero_copy[i][j] = this->zero[i][j];
			if (this->zero[i][j])
				this->num_zero++;
		}

	}
}

void Hungarian::clear_vectors()
{
	this->marked_rows.clear();
	this->marked_cols.clear();
	this->marked_zero.clear();
	this->non_marked_cols.clear();
	this->non_marked_rows.clear();
}

void Hungarian::step2()
{
	while (this->num_zero > 0)
	{
		this->min_zero_row();
	}

	this->num_zero = 0;
	this->mark_matrix();

	for (int i = 0; i < this->marked_cols.size(); i++) {
		this->compare_column_index(this->marked_cols[i]);
	}
}

void Hungarian::min_zero_row()
{
	int min = 0, minp=0;
	int imin = this->non_marked_rows.at(0);
	int position = 0;

	for (int i = 0; i < this->zero[imin].size(); i++) {
		
		if (this->zero[imin][i])
			min++;
	}

	for (int i = 1; i < this->non_marked_rows.size(); i++) {
		minp = 0;
		for (int j = 0; j < this->zero[this->non_marked_rows.at(i)].size(); j++) {
			if (this->zero[this->non_marked_rows.at(i)][j])
				minp ++ ;
		}
		if (minp < min && minp > 0) {
			min = minp;
			imin = this->non_marked_rows.at(i);
			position = i;
		}
	}

	this->non_marked_rows.erase(this->non_marked_rows.begin() + position); 
	this->marked_rows.push_back(imin);

	//mark any 0 elements on the corresponding row and clean up its row and column

	this->find_position(min, imin);
}

void Hungarian::find_position(int min, int imin)
{
	this->num_zero -= min;
	bool one = true;
	double minEl = DBL_MAX;
	Position position_of_min;
	position_of_min.i = imin;
	position_of_min.j = -1;

	if (min > 1) {
		one = false;
		for (int i = 0; i < this->assignments_copy.cols();i++) {
			if (this->assignments_copy(imin, i) < minEl) {
				minEl = this->assignments_copy(imin, i);
				position_of_min.j = i;
			}
		}
	}

	for (int i = 0; i < this->zero_copy[imin].size(); i++) {

		if (this->zero_copy[imin][i]) {
			
			//coordinates of the elements are stored in mark_zero
			min--;
			this->zero_copy[imin][i] = false;

			if (one) {
				Position p;
				p.i = imin;
				p.j = i;
				marked_zero.push_back(p);
				for (int j = 0; j < this->zero_copy.size(); j++) {

					if (this->zero_copy[j][i]) {

						this->zero_copy[j][i] = false;
						//this->num_zero--;

					}
				}
			}
			else {
				if (i == position_of_min.j)
				{
					Position p;
					p.i = imin;
					p.j = i;
					marked_zero.push_back(p);
					for (int j = 0; j < this->zero_copy.size(); j++) {

						if (this->zero_copy[j][i]) {

							this->zero_copy[j][i] = false;
							//this->num_zero--;

						}
					}
				}
			}
		}
		if (min == 0)
			break;
	}	
}

void Hungarian::mark_matrix()
{
	//search row and find out if there are any unmarked 0 elements
	int numEl = this->non_marked_rows.size();
	while (numEl > 0) {	
		int row = this->non_marked_rows.at(numEl-1);
		for (int i = 0; i < this->zero[row].size(); i++) {
				
			if (this->zero[row][i]) {
				this->marked_cols.push_back(i);

				for (int j = 0; j < this->non_marked_cols.size(); j++) {
					if (this->non_marked_cols[j] == i) {

						this->non_marked_cols.erase(this->non_marked_cols.begin() + j);
						break;
					}
				}
			}
		}
		numEl--;
	}	
}

void Hungarian::compare_column_index(const int& column)
{
	for (int i = 0; i < this->marked_zero.size(); i++) {
		if (this->marked_zero[i].j == column) {
			for (int j = 0; j < this->marked_rows.size(); j++) {
				if (this->marked_rows[j] == this->marked_zero[i].i) {
					this->marked_rows.erase(this->marked_rows.begin() + j);
					this->non_marked_rows.push_back(this->marked_zero[i].i);
					break;
				}
			}
		}
	}
}

void Hungarian::identify_the_result()
{
	if((this->marked_rows.size() + this->marked_cols.size()) < min(this->assignments.rows(), this->assignments.cols())) {
			this->adjust_matrix();
	}	
}

void Hungarian::adjust_matrix()
{
	//1. find minimum element for an element that is not in marked_rows and not in marked_cols
	double min = DBL_MAX;

	for (int i = 0; i < this->non_marked_rows.size(); i++) {
		for (int j = 0; j < this->non_marked_cols.size(); j++) {
			if (min > this->assignments(this->non_marked_rows.at(i), this->non_marked_cols.at(j)))
				min = this->assignments(this->non_marked_rows.at(i), this->non_marked_cols.at(j));
		}
	}

	//2. substract the elements which not in marked_rows nor marked_cols from the minimum value
	for (int i = 0; i < this->non_marked_rows.size(); i++) {
		for (int j = 0; j < this->non_marked_cols.size(); j++) {
			this->assignments(this->non_marked_rows.at(i), this->non_marked_cols.at(j)) -= min;
			if (this->assignments(this->non_marked_rows.at(i), this->non_marked_cols.at(j)) == 0)
				this->zero[this->non_marked_rows.at(i)][this->non_marked_cols.at(j)] = true;
		}
	}

	//3. add the element in marked_rows, which is also in marked_cols
	for (int i = 0; i < this->marked_rows.size(); i++) {
		for (int j = 0; j < this->marked_cols.size(); j++) {
			this->assignments(this->marked_rows.at(i), this->marked_cols.at(j)) += min;
		}
	}

	//repeat step 2 and step 3 until conditions satisfy the requirement
	this->clear_vectors();
	this->initial_non_marked_rows();
	this->initial_non_marked_cols();
	this->copy_boolean_matrix();

	this->step2();
	this->identify_the_result();
}

void Hungarian::calculate_the_answer()
{
	for (int i = 0; i < this->marked_zero.size(); i++) {
		cout << "(" << this->marked_zero[i].i << ", " << this->marked_zero[i].j << ")" << endl;
	}

	cout << "Marked-rows" << endl;
	for (int i = 0; i < this->marked_rows.size(); i++) {
		cout << this->marked_rows[i] << " ";
	}

	cout <<endl<< "Marked-cols" << endl;
	for (int i = 0; i < this->marked_cols.size(); i++) {
		cout << this->marked_cols[i] << " ";
	}
	cout << endl;
}

Hungarian::Hungarian(const MatrixXd&  assignments)
{
	
	this->assignments.setZero(assignments.rows(), assignments.cols());
	this->assignments = assignments.replicate(1,1);
	this->assignments_copy = assignments.replicate(1, 1);

	for (int i = 0; i < this->assignments.rows(); i++) {
		vector<bool> pom;
		for (int j = 0; j < this->assignments.cols(); j++) {
			
			pom.push_back(false);
		}
		this->non_marked_rows.push_back(i);
		this->zero.push_back(pom);
		this->zero_copy.push_back(pom);
	}
	this->num_zero = 0;
	this->initial_non_marked_cols();
}

Hungarian::~Hungarian()
{
}

void Hungarian::substract_rows_columns()
{

	double min;
	// substract rows
	for (int i = 0; i < this->assignments.rows(); i++) {
		min = DBL_MAX;
		for (int j = 0; j < this->assignments.cols(); j++) {
			
			if (this->assignments(i, j) < min) {
				min = this->assignments(i, j);
			}
		}

		for (int j = 0; j < this->assignments.cols(); j++) {

			this->assignments(i, j) -= min;

			if (this->assignments(i, j) == 0) {
				this->zero[i][j] = true;
				this->zero_copy[i][j] = true;
				this->num_zero++;
			}
		}
	}

	//substract columns 
	for (int i = 0; i < this->assignments.cols(); i++) {
		
		min = DBL_MAX;
		bool contains = false;

		for (int j = 0; j < this->assignments.rows(); j++) {
			if (this->zero[j][i]) {
				contains = true;
				break;
			}
			if (this->assignments(j, i) < min)
				min = this->assignments(j, i);
		}

		if (!contains) {

			for (int j = 0; j < this->assignments.rows(); j++) {

				this->assignments(j, i) -= min;

				if (this->assignments(j, i) == 0) {
					this->zero[j][i] = true;
					this->zero_copy[j][i] = true;
					this->num_zero++;
				}
			}
		}
	}
	// the process is repeated several times until the elements in the boolean matrix are all false
	this->step2();
	this->identify_the_result();
}

vector<Position> Hungarian::positions()
{
	return this->marked_zero;
}


