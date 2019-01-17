#include "vehicle.h"
#include "json.hpp"
#include <vector>
#include <math.h>
#include "spline.h"
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

using json = nlohmann::json;
using namespace std;

Vehicle::Vehicle(json car_parameters){
    x = car_parameters["x"];
    y = car_parameters["y"];
    s = car_parameters["s"];
    d = car_parameters["d"];
    yaw = car_parameters["yaw"];
    speed = car_parameters["speed"];
    lane_id = 1;
    car_left = false;
    car_right = false;
    car_ahead = false;
    car_ahead_close = false;
    auto previous_path_x = car_parameters["previous_path_x"];
    int prev_size = previous_path_x.size();
    double end_path_s = car_parameters["end_path_s"];
    // Make sure no vehicle at the path planned to avoid collision
    if (prev_size > 0) {
        s = end_path_s;
    }
}

double Vehicle::deg2rad(double x) { return x * M_PI / 180; };
double Vehicle::rad2deg(double x) { return x * 180 / M_PI; };

int Vehicle::laneCalc(double obj_d){
    // Determine the lane of the car base on d
    int obj_lane = -1;
    if ( obj_d > 0 && obj_d < 4 ) {
        obj_lane = 0;
    } else if ( obj_d > 4 && obj_d < 8 ) {
        obj_lane = 1;
    } else if ( obj_d > 8 && obj_d < 12 ) {
        obj_lane = 2;
    }
    return obj_lane;    
}

void Vehicle::check_surrounding(json message){
    auto sensor_fusion = message["sensor_fusion"];
    auto previous_path_x = message["previous_path_x"];
    int prev_size = previous_path_x.size();
    for ( int i = 0; i < sensor_fusion.size(); i++ ) {
        float obj_d = sensor_fusion[i][6];
        int obj_lane = laneCalc(obj_d);
        // Caculate the speed of the vehicle detected
        double vx = sensor_fusion[i][3];
        double vy = sensor_fusion[i][4];
        double obj_speed = sqrt(vx*vx + vy*vy);
        double obj_s = sensor_fusion[i][5];
        // Estimate car s position after executing previous trajectory.
        // Safe distance is 30m for the side cars
        obj_s += ((double)prev_size*0.02*obj_speed);
        if ( obj_lane == lane_id ) { // Car in the same line
            car_ahead |= obj_s > s & obj_s - s < 40; // Car ahead if it's in 40m
            car_ahead_close |= obj_s > s & obj_s - s < 30; // Car is close if it's in 30m
            car_ahead_speed = obj_speed;
            car_ahead_s = obj_s;
        } else if ( obj_lane - lane_id == -1 ) { // Car at left
            car_left |= s - 30 < obj_s && s + 30 > obj_s;
        } else if ( obj_lane - lane_id == 1 ) { // Car at right
            car_right |= s - 30 < obj_s && s + 30 > obj_s;
        }
    }
    cout << car_ahead_close << endl;
};

double Vehicle::behavior_plan(double &ref_v) {
    double speed_diff = 0;
    if (car_ahead_close) { // Car ahead and close
        if ( !car_left && lane_id > 0 ) { // No left car and left lane available
            lane_id--; // Change lane left.
        } else if ( !car_right && lane_id != 2 ){ // No right car and righ lane available
            lane_id++; // Change lane right.
        } else { // Too close and no way to overtake
            speed_diff += (car_ahead_speed-ref_v)/50.*max_acc;
        }
    } else if (car_ahead && car_ahead_speed < 45){ // Car ahead and its speed is slow
        speed_diff += (car_ahead_speed-ref_v)/50.*max_acc + max_acc/2; // Getting closer with a small acceleration
        if (speed_diff > max_acc){ // The acceleration cannot exceed the upper limit
            speed_diff = max_acc;
        }
    } else if (car_ahead && lane_id == 1){ // Car ahead with fast speed and at the cener lane
        speed_diff += (car_ahead_speed-ref_v)/50.*max_acc; // Follow the front car 
    } else { //Otherwise:
        // Back to the center lane
        if ( lane_id != 1 ) { 
            if ( ( lane_id   == 0 && !car_right ) || ( lane_id == 2 && !car_left ) ) {
                lane_id = 1; 
            }
        }
        // Speed up if slow
        if ( ref_v < max_speed ) {
            speed_diff += max_acc;
        }
    }
    return speed_diff;
};

vector<double> Vehicle::getXY(double s, double d, const vector<double> maps_s, const vector<double> maps_x, const vector<double> maps_y){
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-M_PI/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};    
};

void Vehicle::path_gen(json message,double speed_diff, vector<double> &map_waypoints_x, vector<double> &map_waypoints_y, vector<double> &map_waypoints_s, double &ref_v) {
    vector<double> ptsx;
    vector<double> ptsy;
    auto previous_path_x = message["previous_path_x"];
    auto previous_path_y = message["previous_path_y"];
    int prev_size = previous_path_x.size();
    double ref_x = x;
    double ref_y = y;
    double ref_yaw = deg2rad(yaw);

    // Previous points
    if ( prev_size < 2 ) {
        double prev_car_x = x - cos(yaw);
        double prev_car_y = y - sin(yaw);

        ptsx.push_back(prev_car_x);
        ptsx.push_back(x);

        ptsy.push_back(prev_car_y);
        ptsy.push_back(y);
    } else {
        // Use the last two points.
        ref_x = previous_path_x[prev_size - 1];
        ref_y = previous_path_y[prev_size - 1];

        double ref_x_prev = previous_path_x[prev_size - 2];
        double ref_y_prev = previous_path_y[prev_size - 2];
        ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

        ptsx.push_back(ref_x_prev);
        ptsx.push_back(ref_x);

        ptsy.push_back(ref_y_prev);
        ptsy.push_back(ref_y);
    }

    // Setting up target points in the future.
    for (int i = 1; i < 4; i++) {
        vector<double> next_wp = getXY(s + i * 30, 2 + 4*lane_id, map_waypoints_s, map_waypoints_x, map_waypoints_y);
        ptsx.push_back(next_wp[0]);
        ptsy.push_back(next_wp[1]);
    }

    //cout << ptsx.size() << endl;
    // Making coordinates to local car coordinates.
    for (int i = 0; i < ptsx.size(); i++) {
        double shift_x = ptsx[i] - ref_x;
        double shift_y = ptsy[i] - ref_y;

        ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
        ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
    }

    // Create the spline.
    tk::spline sp;
    sp.set_points(ptsx, ptsy);

    // Obtain path points from previous path for continuity.
    for (int i = 0; i < prev_size; i++) {
        next_x_vals.push_back(previous_path_x[i]);
        next_y_vals.push_back(previous_path_y[i]);
    }
    //cout << next_x_vals.size() << endl;

    // Calculate distance y position on 30 m ahead.
    double target_x = 30.0;
    double target_y = sp(target_x);
    double target_dist = sqrt(target_x*target_x + target_y*target_y);

    double x_add_on = 0;

    for( int i = 1; i < 50 - prev_size; i++ ) {
        ref_v += speed_diff;
        //cout  << speed_diff << endl;
        if ( ref_v > max_speed ) {
        ref_v = max_speed;
        } else if ( ref_v < max_acc ) {
        ref_v = max_acc;
        }
        // Obtain the points based on the velocity of the vehicle
        double N = target_dist/(0.02*ref_v/2.24); 
        double x_point = x_add_on + target_x/N;
        double y_point = sp(x_point);

        x_add_on = x_point;

        double x_ref = x_point;
        double y_ref = y_point;

        x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
        y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

        x_point += ref_x;
        y_point += ref_y;

        next_x_vals.push_back(x_point);
        next_y_vals.push_back(y_point);
    }
}