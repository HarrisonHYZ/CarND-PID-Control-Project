#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	this->Kp = Kp;
  	this->Ki = Ki;
  	this->Kd = Kd;
  	p_error = 0.1;
  	i_error = 0.001;
  	d_error = 3.0;
  	update_Kp_flag = true;
  	update_Ki_flag = false;
  	update_Kd_flag = false;
  	best_error = 20.0;
}

void PID::UpdateError(double cte) {
	cte_record_average.push_back(cte);

	if(cte_record_average.size()<10) return;

	double sum = std::accumulate(std::begin(cte_record_average), std::end(cte_record_average), 0.0);
	ave_error = sum / cte_record_average.size();

	std::vector <double>().swap(cte_record_average);

	if(update_Kp_flag){
		cte_record.push_back(ave_error);
		if(cte_record.size()==1){
			if(cte_record[0] < best_error){
				best_error = cte_record[0];
			}
			Kp += p_error;
		}
		if(cte_record.size()==2){
			if(cte_record[1] < best_error){
				best_error = cte_record[1];
				p_error *= 1.1;
				update_Kp_flag = false;
				update_Ki_flag = true;
				std::vector <double>().swap(cte_record);
			}
			else{
				Kp -= 2 * p_error;
			}
		}
		if(cte_record.size()==3){
			if(cte_record[2] < best_error){
				best_error = cte_record[2];
				p_error *= 1.1;
				update_Kp_flag = false;
				update_Ki_flag = true;
				std::vector <double>().swap(cte_record);
			}
			else{
				Kp += p_error;
				p_error *= 0.9;
				update_Kp_flag = false;
				update_Ki_flag = true;
				std::vector <double>().swap(cte_record);
			}
		}
	}
	if(update_Ki_flag){
		cte_record.push_back(ave_error);
		if(cte_record.size()==1){
			if(cte_record[0] < best_error){
				best_error = cte_record[0];
			}
			Ki += i_error;
		}
		if(cte_record.size()==2){
			if(cte_record[1] < best_error){
				best_error = cte_record[1];
				i_error *= 1.1;
				update_Ki_flag = false;
				update_Kd_flag = true;
				std::vector <double>().swap(cte_record);
			}
			else{
				Ki -= 2 * i_error;
			}
		}
		if(cte_record.size()==3){
			if(cte_record[2] < best_error){
				best_error = cte_record[2];
				i_error *= 1.1;
				update_Ki_flag = false;
				update_Kd_flag = true;
				std::vector <double>().swap(cte_record);
			}
			else{
				Ki += i_error;
				i_error *= 0.9;
				update_Ki_flag = false;
				update_Kd_flag = true;
				std::vector <double>().swap(cte_record);
			}
		}
	}
	if(update_Kd_flag){
		cte_record.push_back(ave_error);
		if(cte_record.size()==1){
			if(cte_record[0] < best_error){
				best_error = cte_record[0];
			}
			Kd += d_error;
		}
		if(cte_record.size()==2){
			if(cte_record[1] < best_error){
				best_error = cte_record[1];
				d_error *= 1.1;
				update_Kd_flag = false;
				update_Kp_flag = true;
				std::vector <double>().swap(cte_record);
			}
			else{
				Kd -= 2 * d_error;
			}
		}
		if(cte_record.size()==3){
			if(cte_record[2] < best_error){
				best_error = cte_record[2];
				d_error *= 1.1;
				update_Kd_flag = false;
				update_Kp_flag = true;
				std::vector <double>().swap(cte_record);
			}
			else{
				Kd += d_error;
				d_error *= 0.9;
				update_Kd_flag = false;
				update_Kp_flag = true;
				std::vector <double>().swap(cte_record);
			}
		}
	}
}

double PID::TotalError() {
	return (p_error + i_error + d_error);
}

