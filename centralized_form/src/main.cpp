#include <iostream>
#include <ros/ros.h>
#include "vehicle.h"
#include "utility.h"
#include "batch_solver.h"
#include <visualization_msgs/Marker.h>
#include <std_msgs/Bool.h>
#include <memory>
size_t N = 10;//求解输出预测长度
size_t m = 3;//车辆数
size_t Hz = 10;
size_t shift = 0;
double xorigin, yorigin, thetaorigin;
std::vector<double> xref;
std::vector<double> yref;
std::vector<double> thetaref;
std::vector<double> vref;
double d = 0.4;
std::vector<double> xinit;
std::vector<double> yinit;
std::vector<double> thetainit;
double ts = 0.2;
double safety_dist = 0.7;
double distanceThreshold = safety_dist * 0.5;
bool solve_success = false;
std::vector<std::shared_ptr<Vehicle>> vehicles;
std::vector<std::vector<double>> obst;
std::vector<std::vector<std::vector<double>>> goals;
std::shared_ptr<BatchSolver> bs(new BatchSolver(N, m, xref, yref, thetaref, vref, d, xinit, yinit, thetainit, ts, safety_dist * safety_dist, obst));
std::vector<std::vector<std::vector<double>>> pre_states(m, std::vector<std::vector<double>>(N + 1, std::vector<double>(3, 0.0)));
std::vector<std::vector<std::vector<double>>> pre_inputs(m, std::vector<std::vector<double>>(N + 1, std::vector<double>(2, 0.0)));
//ros::Publisher vehicle_pub;
ros::Publisher markerArray;

void RunMPC();
void UpdateNeighborsPos();
void UpdateRef();
void EnableDockingMode();
void Initialize(ros::NodeHandle& n);


int main(int argc, char* argv[]) {
    ros::init(argc, argv, "centralized");
    ros::NodeHandle n;
    ros::Rate loop_rate(Hz);
    Initialize(n);
    //std::cout<<"test"<<std::endl;
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
        RunMPC();
        //exit(0);
    }
    return 0;
};

void Initialize(ros::NodeHandle& n) {

    xinit.push_back(0.0);yinit.push_back(1.0);thetainit.push_back(3.14 / 2.0);
    xinit.push_back(-3.0);yinit.push_back(-3.0);thetainit.push_back(3.14 / 2.0);
    xinit.push_back(3.0);yinit.push_back(-3.0);thetainit.push_back(3.14 / 4.0);
    //xinit.push_back(3.0);yinit.push_back(0.0);thetainit.push_back(-3.14);
    //xinit.push_back(0.0);yinit.push_back(-3.0);thetainit.push_back(3.14/2.0);
    bs->set_initial_states(xinit, yinit, thetainit);

    xref.push_back(8.0);yref.push_back(0.0);thetaref.push_back(0.0);vref.push_back(0.4);
    xref.push_back(7.0);yref.push_back(-1.0);thetaref.push_back(0.0);vref.push_back(0.4);
    xref.push_back(7.0);yref.push_back(1.0);thetaref.push_back(0.0);vref.push_back(0.4);

    xorigin = 8.0;yorigin = 0.0;thetaorigin = 0.0;

    std::vector<std::vector<double>> goal;
    goal.push_back({ 0,0,0 });goal.push_back({ -1.0,-1.0,0 });goal.push_back({ -1.0,1.0,0 });
    goals.push_back(goal);
    goal.clear();
    goal.push_back({ 4,0,0 });goal.push_back({ 4.0,-2.0,0 });goal.push_back({ 4.0,2.0,0 });
    goals.push_back(goal);
    goal.clear();
    goal.push_back({ 9,0,0 });goal.push_back({ 8,0.0,0 });goal.push_back({ 10,0.0,0 });
    goals.push_back(goal);
    goal.clear();
    goal.push_back({ 9,0,0 });goal.push_back({ 8.3,0.0,0 });goal.push_back({ 9.7,0.0,0 });
    goals.push_back(goal);
    goal.clear();

    //xref.push_back(-3.0);yref.push_back(0.0);thetaref.push_back(0.0);
    //xref.push_back(0.0);yref.push_back(2.0);thetaref.push_back(0.0);

    assert(xinit.size() == m and yinit.size() == m and thetainit.size() == m);
    assert(xref.size() == m and yref.size() == m and thetaref.size() == m);
    bs->set_ref_states(xref, yref, thetaref);
    bs->set_ref_states(vref);

    // std::vector<double> obst1 = { 0.0,4.0 };
    // obst.push_back(obst1);
    // obst1 = { 0.0,2.0 };
    // obst.push_back(obst1);
    // obst1 = {0.0,-1.0};
    // obst.push_back(obst1);
    // bs->set_obst_(obst);

    for (size_t i = 0; i < m; i++) {
        std::shared_ptr<Vehicle> v(new Vehicle(n, i + 1, xinit[i], yinit[i], thetainit[i], ts, d));
        vehicles.push_back(v);
    }

    //vehicle_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    markerArray = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 10);
    sleep(3);//wait for rviz init
};


void RunMPC() {
    ShowObstacleInRviz(obst, safety_dist, markerArray);

    if (solve_success) {
        for (int i = 0; i < m; i++) {
            vehicles[i]->UpdateStates(pre_states[i][0][0], pre_states[i][0][1], pre_states[i][0][2], pre_inputs[i][0][0], pre_inputs[i][0][1]);
        }
        solve_success = false;
        shift = 0;
    }
    else {
        shift++;
        std::cout << " ***Solve failed! Use former input " << shift << " ***" << std::endl;
        if (shift < N) {
            for (int i = 0; i < m; i++) {
                vehicles[i]->UpdateStates(pre_states[i][shift][0], pre_states[i][shift][1], pre_states[i][shift][2], pre_inputs[i][shift][0], pre_inputs[i][shift][1]);
            }
        }
        else {
            std::cout << " *** !!! no more former input !!! *** " << std::endl;
            return;
        }
    }


    for (int i = 0; i < m; i++) {
        xinit[i] = vehicles[i]->get_x();
        yinit[i] = vehicles[i]->get_y();
        thetainit[i] = vehicles[i]->get_theta();
    }

    ShowVehicleInRviz(xinit, yinit, thetainit, safety_dist, markerArray);
    bs->set_initial_states(xinit, yinit, thetainit);
    UpdateRef();//update goal
    ShowRefPoint(xref, yref, thetaref, markerArray);
    bs->set_ref_states(xref, yref, thetaref);
    solve_success = bs->Solve(pre_states, pre_inputs);

    // for (int i = 0; i < m; i++) {
    //     for (int j = 0; j <= N; j++) {
    //         std::cout<<"car "<<i<<" step "<<j<<" : "<< pre_inputs[i][j][0]<<" "<< pre_inputs[i][j][1]<<std::endl;
    //     }
    // };

    TrajRviz(pre_states, safety_dist, markerArray);
    return;
}
void UpdateRef() {
    auto checkGoal = [&]() {
        double dismax = 0;
        for (int i = 0;i < m;i++) {
            dismax = std::max(dismax, dis2d(xinit[i], yinit[i], xref[i], yref[i]));
        }
        ROS_INFO("dismax: %lf", dismax);
        return dismax < distanceThreshold;
        };
    static int status = 0;
    if (checkGoal()) {
        ROS_INFO("change status: %d", status);
        if (status >= goals.size()) {
            status = goals.size() - 1;
            EnableDockingMode();
            auto spdlim = 0.2;
            vref.clear();
            vref.push_back(spdlim);vref.push_back(spdlim);vref.push_back(spdlim);
            bs->set_ref_states(vref);
        }
        for (int i = 0;i < m;++i) {
            xref[i] = xorigin + goals[status][i][0];
            yref[i] = yorigin + goals[status][i][1];
            thetaref[i] = thetaorigin + goals[status][i][2];
            // ROS_INFO("car %d: %lf, %lf, %lf", i, xref[i], yref[i], thetaref[i]);
        }
        status++;
    }
    return;
}

void EnableDockingMode() {
    std_msgs::Bool msg;
}