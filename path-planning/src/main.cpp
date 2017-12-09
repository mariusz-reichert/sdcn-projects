#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"

using namespace std;

// for convenience
using json = nlohmann::json;

namespace {
// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }

constexpr double VELOCITY_INC = 2.0 * 0.214;
constexpr double SPEED_LIMIT = 49.5;
constexpr double BUFFER_VELOCITY = 1.0;
constexpr uint kPORT = 4567;

using LaneCost = double;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos and b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(
  double s, 
  double d, 
  const vector<double>& maps_s, 
  const vector<double>& maps_x, 
  const vector<double>& maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] and (prev_wp < (int)(maps_s.size()-1) )){
		prev_wp++;
	}

	const int wp2 = (prev_wp+1)%maps_x.size();
	const double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	
  // the x,y,s along the segment
	const double seg_s = (s-maps_s[prev_wp]);
	const double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	const double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	const double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

// SOLUTION BEGIN

struct Action {
  Action(int lane_, double ref_vel_)
  : lane{lane_}
  , ref_vel{ref_vel_}
  {}
  Action() = default;
  int lane{};
  double ref_vel{};
};

Action choose_action(
  double velocity, 
  int lane, 
  const vector<vector<int>>& sensor_fusion, 
  int prev_size, 
  double car_s) {

  // car in front avoidance start
  // go through sensor fusion and see if a car is in front of us
  Action next_action{};
  next_action.ref_vel = velocity;
  next_action.lane = lane;
  static double capped_speed = 0.0;

  vector<LaneCost> laneCosts{0.0,0.0,0.0};

  for (size_t i = 0; i < sensor_fusion.size(); ++i) {
    const double vx = sensor_fusion[i][3];
    const double vy = sensor_fusion[i][4];
    const double check_speed = sqrt(vx*vx+vy*vy);
    double check_car_s = sensor_fusion[i][5];
    float d = sensor_fusion[i][6];

    for (size_t i =0; i < 3; ++i) {
      if(d < (2+4*i+2) and d > (2+4*i-2)) {
        // if using previous points can project s value outwards in time
        // it is needed cause we're using previous points not the exact current poistion of car
        // so from previous point point of view we'are looking where the
        // car is in the future
        check_car_s += ((double)prev_size*.02*check_speed);
        if ((check_car_s > car_s) and 
            ((check_car_s-car_s) < 30) and 
            (check_speed < velocity+VELOCITY_INC))
        {
          laneCosts[i] += 1.0;
          capped_speed = check_speed;
        }
        // revert s value projection
        check_car_s -= ((double)prev_size*.02*check_speed);
        if ((check_car_s <= car_s) and ((car_s-check_car_s) < 30))
        {
          laneCosts[i] += 1.0;
          capped_speed = check_speed;
        }
      }
    }
  }

  auto increase_velocity = [&](){
    if ((next_action.ref_vel + VELOCITY_INC) < SPEED_LIMIT)
      next_action.ref_vel += VELOCITY_INC;
  };

  if (laneCosts[lane] == 0.0) {
    // continue driving cost free lane
    increase_velocity();
  }
  else if (lane - 1 >=0 and laneCosts[lane - 1] == 0.0) {
    // try lane on the left if current one is not cost free
    increase_velocity();
    next_action.lane = lane - 1;
  }
  else if (lane + 1 <=2 and laneCosts[lane + 1] == 0.0) {
    // try lane on the right if current one and on the left are not cost free
    increase_velocity();
    next_action.lane = lane + 1;
  }
  else if (all_of(begin(laneCosts), 
                  end(laneCosts), 
                  [](const LaneCost&c){ return c != 0.0; })) {
    // if there is no cost free lane follow capped velocity

    if ((next_action.ref_vel + VELOCITY_INC) < capped_speed)
      next_action.ref_vel += VELOCITY_INC;
    else
      next_action.ref_vel -= VELOCITY_INC;
  }
  else {
    // better slow down in another cases
    next_action.ref_vel -= VELOCITY_INC;
  }

  return next_action;
}

// SOLUTION END

}

int main(int argc, char* argv[]) {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x{};
  vector<double> map_waypoints_y{};
  vector<double> map_waypoints_s{};
  vector<double> map_waypoints_dx{};
  vector<double> map_waypoints_dy{};

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  ifstream in_map(map_file_.c_str(), ifstream::in);

  string line{};
  while (getline(in_map, line)) {
  	istringstream iss(line);
  	double x, y;
  	float s, d_x, d_y;
  	iss >> x >> y >> s >> d_x >> d_y;

  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }

  int lane = 1;
  double ref_vel = 0.0;

  h.onMessage([&](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length and length > 2 and data[0] == '4' and data[1] == '2') {
      const auto s = hasData(data);

      if (s != "") {
        const auto j = json::parse(s);
        const string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // SOLUTION BEGIN
          // Main car's localization Data
          const double car_x = j[1]["x"];
          const double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          const double car_d = j[1]["d"];
          const double car_yaw = j[1]["yaw"];
          const double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          const auto& previous_path_x = j[1]["previous_path_x"];
          const auto& previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          const double end_path_s = j[1]["end_path_s"];
          const double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          const auto& sensor_fusion = j[1]["sensor_fusion"];

          const size_t prev_size = previous_path_x.size();

          if (prev_size > 0) {
            // assume car s is at the end of lastly generated path
            car_s = end_path_s;
          }

          const auto action = choose_action(ref_vel, lane, sensor_fusion, prev_size, car_s);
          ref_vel = action.ref_vel;
          lane = action.lane;

          // create a list of widely spaced (x,y) waypoints, evenly spaced at 30m
          // later we will interpolate these points with spline and fill it in with
          // more points that control speed
          vector<double> ptsx{};
          vector<double> ptsy{};
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          // either we will reference the starting point as where the car is 
          // or at the previous path end points
          // if the previous size is almost empty use the car as the starting points
          if (prev_size < 2) {
            const double prev_car_x = car_x - cos(car_yaw);
            const double prev_car_y = car_y - sin(car_yaw);

            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);

            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          } 
          // use the previous path's end point as a starting reference
          else {
            // redefine reference state as previous path end point
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];
            const double ref_x_prev = previous_path_x[prev_size-2];
            const double ref_y_prev = previous_path_y[prev_size-2];
            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

            // use two points that make the path tangent to the previous path's end point
            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);

            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }

          // using above 2 reference points lets generate next ones:
          // In Frenet add evenly 30m spaced points ahead of the starting reference
          const auto next_wp0 = getXY(car_s+30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          const auto next_wp1 = getXY(car_s+60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          const auto next_wp2 = getXY(car_s+90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);

          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);

          // lets shift the points we have so far into car reference frame
          for (size_t i = 0; i < ptsx.size(); ++i) {
            const auto shift_x = ptsx[i]-ref_x;
            const auto shift_y = ptsy[i]-ref_y;

            ptsx[i] = (shift_x * cos(0-ref_yaw)-shift_y * sin(0-ref_yaw));
            ptsy[i] = (shift_x * sin(0-ref_yaw)+shift_y * cos(0-ref_yaw));
          }

          // create a spline
          tk::spline s{};
          // set x,y points to the spline
          s.set_points(ptsx, ptsy);

          // define the actual (x,y) points we will use for the planner
          vector<double> next_x_vals{};
          vector<double> next_y_vals{};

          // start with all of the previous path points from last time
          for (size_t i = 0; i < previous_path_x.size(); ++i) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          // calculate how to break up spline points so that we travel at our desired reference velocity
          const double target_x = 30.0; // the horizon, we're interpolating 30m ahead
          const double target_y = s(target_x);
          // linearization here:
          const double target_dist = sqrt(target_x*target_x + target_y * target_y);

          double x_add_on = 0;

          // fill up the rest of our path planner after filling it with previous points. here we will
          // always output 50 points
          for (size_t i = 1; i <= 50-previous_path_x.size(); ++i) {
            const double N = target_dist/(.02*ref_vel/2.24); // 2.24 to convert from miles/h to m/s, 0.2 is time pre simulation step
            double x_point = x_add_on+(target_x)/N;
            double y_point = s(x_point);

            x_add_on = x_point;

            const auto x_ref = x_point;
            const auto y_ref = y_point;

            // rotate back from car coordinates
            x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
            y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));
            // and make shift
            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }

          // SOLUTION END
          
          json msgJson{};
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          const auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  if (h.listen(kPORT)) {
    std::cout << "Listening to port " << kPORT << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
















































































