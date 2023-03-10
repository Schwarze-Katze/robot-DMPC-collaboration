/**
* This code doesnt consider the time delay caused by the communication and the solving process;
* The failure to solve is considered but it is assumed the time it costs is ignorable even if it fails to solve;
**/

#include "batch_solver.h"
#include <iostream>
#include "ros/ros.h"
#include "vehicle.h"
#include "utility.h"
#include <visualization_msgs/Marker.h>
#include <memory>
#include <thread>
#include <mutex>
#include <atomic>
#include <numeric>
#include "mymsg/neighborpos.h"
#include "mymsg/refpos.h"
#include "std_msgs/String.h"


const int vehicleId = 2;
size_t N = 15;
size_t m = 2;// represent the num of neighbors for this vehicle
std::vector<int> neigId;
std::vector<bool> first_solution;
double Hz = 5.0;
size_t update_shift = 0;

double xr;
double yr;
double thetar;
const double disteps = 0.1;

std::mutex neig_mtx;

double xinit;
double yinit;
double thetainit;

// ******protected by neig_mtx
std::vector<std::vector<std::vector<double>>> neig;
std::vector<std::vector<double>> pre_states(N + 1, std::vector<double>(3.0, 0.0));
std::vector<std::vector<double>> pre_inputs(N + 1, std::vector<double>(2.0, 0.0));


bool solve_success = false;

// each vehicle may have different solving time for the intial solution;
// some of them might be quite long; So these flags are used for synchronous purpose;

double d = 0.5;
double ts = 1.0 / Hz;
double safety_dist = 0.5;


std::shared_ptr<Vehicle> vehicle;
std::vector<std::vector<double>> obst;
std::shared_ptr<BatchSolver> bs(new BatchSolver(N, xr, yr, thetar, d, xinit, yinit, thetainit, ts, safety_dist, obst, neig));

ros::Publisher vehicle_pub;// for visulization
ros::Publisher markerArray;
ros::Publisher neig_pub;// tells other robots my pos
std::vector<ros::Subscriber> subs;
ros::Subscriber subRef;
//ros::Subscriber testsub;

void UpdateVisualize();
void UpdateNeighborsPos();
void Initialize(ros::NodeHandle& n);
void NeighborCallback(const mymsg::neighborpos& msg);
void UpdateReference(const mymsg::refpos& msg);

int main(int argc, char* argv[]) {
	ros::init(argc, argv, "vehicle02");
	ros::NodeHandle n;
	ros::Rate loop_rate(50);
	Initialize(n);


	std::thread sim_thread(&UpdateVisualize);
	sim_thread.detach();


	while (ros::ok()) {
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
};

void Initialize(ros::NodeHandle& n) {
	neigId.resize(m);
	std::iota(neigId.begin(), neigId.end(), 1);
	for (int i = vehicleId - 1;i < m;++i) {
		++neigId[i];
	}
	first_solution = std::vector<bool>(m + 1, false);
	first_solution[vehicleId - 1] = true;
#if 1
	// test setting 1
	xinit = 0.0;yinit = 10.0;thetainit = 0.0;
	xr = 0.0;yr = 0.0;thetar = 0.0;
	std::vector<double> obst1 = { 0.0,15.0 };
	std::vector<std::vector<double>> neig1(N + 1, std::vector<double>{-5.0, 0.0});
	std::vector<std::vector<double>> neig2(N + 1, std::vector<double>{0.0, 5.0});
#endif

#if 0
	// test setting 2
	xinit = -5.0;yinit = -5.0;thetainit = 3.14 / 4.0;
	xr = 5.0;yr = 5.0;thetar = 0.0;
	std::vector<double> obst1 = { 0.0,0.0 };
	std::vector<std::vector<double>> neig1(N + 1, std::vector<double>{-5.0, 5.0});
	std::vector<std::vector<double>> neig2(N + 1, std::vector<double>{5.0, -5.0});
#endif

#if 0
	// test setting 3
	xinit = -5.0;yinit = -1.0;thetainit = 0.00001;
	xr = 5.0;yr = -1.0;thetar = 0.0;
	std::vector<double> obst1 = { 0.0,0.0 };
	std::vector<std::vector<double>> neig1(N + 1, std::vector<double>{-5.0, 1.0});
	std::vector<std::vector<double>> neig2(N + 1, std::vector<double>{5.0, 0.0});
#endif

#if 0
	// test setting for formation 1
	xinit = -2.0;yinit = -6.0;thetainit = 0.00001;
	xr = -1.0;yr = 4.0;thetar = 0.0;
	std::vector<double> obst1 = { 0.0,0.0 };
	std::vector<std::vector<double>> neig1(N + 1, std::vector<double>{0.0, -3.0});
	std::vector<std::vector<double>> neig2(N + 1, std::vector<double>{2.0, 6.0});
#endif 

#if 0
	// test setting for formation 2
	xinit = 0.0;yinit = 10.0;thetainit = 0;
	xr = 0;yr = 0;thetar = 0.0;
	std::vector<double> obst1 = { 0.0,15.0 };
	std::vector<std::vector<double>> neig1(N + 1, std::vector<double>{0.0, -3.0});
	std::vector<std::vector<double>> neig2(N + 1, std::vector<double>{4.0, 8.0});
#endif 

#if 0
	// test setting for formation 3
	xinit = -0.0;yinit = -7.0;thetainit = 0.1;
	xr = -1.0;yr = 4.0;thetar = 0.0;
	std::vector<double> obst1 = { 0.0,0.0 };
	std::vector<std::vector<double>> neig1(N + 1, std::vector<double>{-3.0, -7.0});
	std::vector<std::vector<double>> neig2(N + 1, std::vector<double>{3.0, -7.0});
#endif 
	obst.push_back(obst1);
	neig.push_back(neig1);
	neig.push_back(neig2);
	assert(neig.size() == m);
	bs->set_obst_(obst);
	bs->set_ref_states(xr, yr, thetar);
	bs->set_initial_states(xinit, yinit, thetainit);
	bs->set_neighbors(neig, neig_mtx);
	std::shared_ptr<Vehicle> vehicle_tmp(new Vehicle(xinit, yinit, thetainit, ts, d));
	vehicle = vehicle_tmp;

	vehicle_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
	markerArray = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 10);
	neig_pub = n.advertise<mymsg::neighborpos>("neig" + std::to_string(vehicleId) + "pos", 10);
	for (auto id : neigId) {
		subs.push_back(n.subscribe("neig" + std::to_string(id) + "pos", 1000, NeighborCallback));
	}
	subRef = n.subscribe("formation", 1000, UpdateReference);
	//subRef = n.subscribe("refPointPos", 1000, refCallback);
	// testsub = n.subscribe("chatter", 1000, chatterCallback);

};
void UpdateVisualize() {

	ros::Rate loop_rate_sim(Hz);

	ros::spinOnce();
	loop_rate_sim.sleep();

	while (ros::ok()) {

		/**
		* in the instant i ,this vehicle solves the optimization problem using assumed trajectory obtained in
		* the last iteration. Then it sends the solution immediately. Then it sleeps for a while.
		* The states is updated in the instant i+1;
		**/
		ObstRviz(obst, safety_dist, markerArray);

		bs->set_initial_states(xinit, yinit, thetainit);
		bs->set_ref_states(xr, yr, thetar);
		bs->set_neighbors(neig, neig_mtx);
		bs->Solve(pre_states, pre_inputs, solve_success);
		std::cout << "V02-init:" << xinit << ',' << yinit << "\tV02-ref:" << xr << ',' << yr << std::endl;
		//Status publishing
		if (solve_success) {
			update_shift = 0;
			std::vector<double> pre_x;
			std::vector<double> pre_y;
			for (int i = 0; i < pre_states.size(); i++) {
				pre_x.push_back(pre_states[i][0]);
				pre_y.push_back(pre_states[i][1]);
			}
			mymsg::neighborpos msg;
			msg.xpos = pre_x;
			msg.ypos = pre_y;
			msg.time_stamp = ros::Time::now().toSec();
			msg.id = vehicleId;
			neig_pub.publish(msg);
		}
		else {// if fail to solve, publish the shifted pre_states
			update_shift++;
			// std::cout<<" ***publish previous states : "<< update_shift <<" ***"<<std::endl;		
			if (update_shift < N) {
				std::vector<double> pre_x;
				std::vector<double> pre_y;
				for (int i = update_shift; i < pre_states.size(); i++) {
					pre_x.push_back(pre_states[i][0]);
					pre_y.push_back(pre_states[i][1]);
				}

				for (int i = pre_states.size(); i < pre_states.size() + update_shift; i++) {
					pre_x.push_back(0.0);
					pre_y.push_back(0.0);
				}
				mymsg::neighborpos msg;
				msg.xpos = pre_x;
				msg.ypos = pre_y;
				msg.time_stamp = ros::Time::now().toSec();
				msg.id = vehicleId;
				neig_pub.publish(msg);
			}
			else {
				// std::cout<<" *** !!! no more previous states !!! *** "<<std::endl;
				return;
			}
		}

		ros::spinOnce();// send the solution ASAP after the solving
		loop_rate_sim.sleep();
		for (bool tmp = true;tmp;tmp = true) {
			for (int i = 0;i < first_solution.size();++i) {
				tmp &= first_solution[i];
			}
		}
		if (solve_success) {
			solve_success = false;
			vehicle->UpdateStates(pre_inputs[0][0], pre_inputs[0][1]);
			// std::cout<<"1:vl="<<pre_inputs[0][0]<<",1:vr="<<pre_inputs[0][1]<<std::endl;
		}
		else {// if fail to solve, use shifted input in the last iteration
			if (update_shift < N) {
				vehicle->UpdateStates(pre_inputs[update_shift][0], pre_inputs[update_shift][1]);
				// std::cout<<"1:vl="<<pre_inputs[update_shift][0]<<"1:vr="<<pre_inputs[update_shift][1]<<std::endl;
			}
			else {
				return;
			}
		}
		/*auto xcur=vehicle->get_x();
		auto ycur = vehicle->get_y();
		auto thetacur = vehicle->get_theta();
		double dist = pow(xcur - xr, 2) + pow(ycur - yr, 2);
		if(dist<disteps){

		}
		else{}*/
		xinit = vehicle->get_x();
		yinit = vehicle->get_y();
		thetainit = vehicle->get_theta();
		VehicleRviz(xinit, yinit, thetainit, safety_dist, vehicle_pub);
		HeadingRviz(xinit, yinit, thetainit, safety_dist, vehicle_pub);
		TrajRviz(pre_states, safety_dist, markerArray);
	}

};

void NeighborCallback(const mymsg::neighborpos& msg) {
	int id = msg.id - 1;//id=0...m
	assert(id >= 0 and id < m + 1);
	std::vector<double> x = msg.xpos;
	std::vector<double> y = msg.ypos;
	std::lock_guard<std::mutex> lk(neig_mtx);
	for (size_t i = 1; i < neig[0].size();i++) {
		neig[id][i - 1][0] = x[i];
		neig[id][i - 1][1] = y[i];
	}
	neig[id][neig[id].size() - 1][0] = neig[id][neig[id].size() - 2][0];
	neig[id][neig[id].size() - 1][1] = neig[id][neig[id].size() - 2][1];
	first_solution[id] = true;
};

//void refCallback(const mymsg::refpos& msg) {

//};

void UpdateReference(const mymsg::refpos& msg) {
	xr = msg.xr[vehicleId - 1];
	yr = msg.yr[vehicleId - 1];
	thetar = msg.thetar[vehicleId - 1];
	std::cout << "ref:" << xr << ',' << yr << ',' << thetar << std::endl;
}
