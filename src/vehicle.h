#include "json.hpp"
#include <vector>
using json = nlohmann::json;
using namespace std;

class Vehicle{
    public:
        double x;
        double y;
        double s;
        double d;
        double yaw;
        double speed;
        int lane_id;
        bool car_left;
        bool car_right;
        bool car_ahead;
        bool car_ahead_close;
        double car_ahead_speed;
        double car_ahead_s;
        vector<double> next_x_vals;
        vector<double> next_y_vals;
        const double max_speed = 49.5;
        const double max_acc = 0.224;


    Vehicle(json car_parameters);

    double deg2rad(double x);
    double rad2deg(double x);

    // Check the cars in the surrounding by based on the result of sensor fusion
    void check_surrounding(json message);

    // Calculate the lane id 
    int laneCalc(double obj_d);

    // Behavioral planning
    double behavior_plan(double &ref_v);

    // Transform from Frenet s,d coordinates to Cartesian x,y
    vector<double> getXY(double s, double d, const vector<double> maps_s, const vector<double> maps_x, const vector<double> maps_y);

    // Path planning
    void path_gen(json message, double speed_diff, vector<double> &map_waypoints_x, vector<double> &map_waypoints_y, vector<double> &map_waypoints_s, double &ref_v);
};