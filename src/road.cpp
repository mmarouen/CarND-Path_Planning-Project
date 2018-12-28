#include <iostream>
#include "road.h"
#include "vehicle.h"
#include <math.h>
#include <map>
#include <string>
#include <iterator>
#include <cmath>
#include <vector>
#include "Eigen/Dense"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;


/*
 * Initializes Road
 */
Road::Road(float speed_limit, vector<float> lane_speeds,int lane_width, float time_horizon,float mph_convert) {

    this->num_lanes = lane_speeds.size();
    this->lane_speeds = lane_speeds;
    this->speed_limit = speed_limit;
    this->lane_width=lane_width;
    this->time_horizon=time_horizon;
    this->mph_convert=mph_convert;
}

Road::~Road() {}

Vehicle Road::get_ego() {

	return this->vehicles.find(this->ego_key)->second;
}

void Road::populate_traffic2(vector<vector<double>> sf_data,vector<double> car_data) {
	Vehicle mycar=this->get_ego();
	this->vehicles_added=0;
	this->vehicles.clear();
	for (int i = 0; i < sf_data.size(); i++){
		vector<double> car=sf_data[i];
      	double x = car[1];
      	double y = car[2];
      	double vx = car[3];
      	double vy = car[4];
      	double s = car[5];
      	double d = car[6];
      	int lane=d/lane_width;
      	double speed=sqrt(vx*vx+vy*vy);
		Vehicle vehicle = Vehicle(lane,s,d,speed,0,"CS");
		this->vehicles_added += 1;
		this->vehicles.insert(std::pair<int,Vehicle>(vehicles_added,vehicle));
	}
	vector<float> ego_conf={this->speed_limit*this->mph_convert,this->num_lanes,mycar.goal_s,mycar.max_acceleration};
	int lane_num=car_data[3]/this->lane_width;
	this->add_ego2(lane_num,car_data[2],car_data[3],car_data[4],car_data[5],car_data[6],car_data[7],ego_conf);
}

void Road::advance() {

	map<int ,vector<Vehicle> > predictions;
	map<int, Vehicle>::iterator it = this->vehicles.begin();
	float current_speed=0;

    while(it != this->vehicles.end()){
        int v_id = it->first;
        if(v_id != ego_key){
            vector<Vehicle> preds = it->second.generate_predictions(time_horizon);
            predictions[v_id] = preds;
        }
        it++;
    }
	it = this->vehicles.begin();
	while(it != this->vehicles.end()){
    	int v_id = it->first;
        if(v_id == ego_key){
        	Vehicle mycar=this->get_ego();
        	if(mycar.lane==mycar.target_lane){
            	vector<Vehicle> trajectory = it->second.choose_next_state(predictions,time_horizon);
            	it->second.realize_next_state(trajectory);
        	}else{
        		vector<float> kinematics=mycar.get_kinematics(predictions,mycar.target_lane,time_horizon);
        		it->second.s=kinematics[0];
        		it->second.v_s=kinematics[1];
        		it->second.a_s=kinematics[2];
        		it->second.lane=mycar.target_lane;
        		it->second.d=4*mycar.target_lane+2;
        		/*double delta_d=1;
        		if(mycar.target_lane<mycar.lane) delta_d=-2;
        		it->second.d+=delta_d;*/
        	}
        }else{
            it->second.s_increment(time_horizon);
        }
        it++;
    }
}

void Road::add_ego2(int lane_num, float s,float d,float v,float a,int state_of_car,int target_lane, vector<float> config_data) {
	string car_state="KL";
	if(state_of_car==1) car_state="LCR";
	if(state_of_car==-1) car_state="LCL";

    Vehicle ego = Vehicle(lane_num, s,d, v, a,car_state,target_lane);
    ego.configure(config_data);
    this->vehicles.insert(std::pair<int,Vehicle>(ego_key,ego));
}

vector<double> Road::JMT(vector< double> start, vector <double> end, double T)
{

    MatrixXd A = MatrixXd(3, 3);
	A << T*T*T, T*T*T*T, T*T*T*T*T,
			    3*T*T, 4*T*T*T,5*T*T*T*T,
			    6*T, 12*T*T, 20*T*T*T;

	MatrixXd B = MatrixXd(3,1);
	B << end[0]-(start[0]+start[1]*T+.5*start[2]*T*T),
			    end[1]-(start[1]+start[2]*T),
			    end[2]-start[2];

	MatrixXd Ai = A.inverse();
	MatrixXd C = Ai*B;
	vector <double> result = {start[0], start[1], .5*start[2]};
	for(int i = 0; i < C.size(); i++)
	{
	    result.push_back(C.data()[i]);
	}

    return result;

}

