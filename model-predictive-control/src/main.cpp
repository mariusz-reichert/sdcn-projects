#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <algorithm>

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.hpp"
#include "json.hpp"

using namespace model_predictive_control_project;

// for convenience
using json = nlohmann::json;
using Eigen::VectorXd;
using Eigen::Map;
using std::endl;
using std::cout;

namespace {

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(const string& s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(const VectorXd& coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); ++i) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
VectorXd polyfit(const VectorXd& xvals, const VectorXd& yvals, int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); ++i) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); ++j) {
    for (int i = 0; i < order; ++i) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  return Q.solve(yvals);
}

constexpr double kREF_ANGLE = 0.0; // reference coordinate system angle
constexpr double kREF_X = 0.0; // reference coordinate system x position
constexpr double kREF_Y = 0.0; // reference coordinate system y position
constexpr int kPOLY_ORDER = 3;
constexpr double kLF = 2.67;
constexpr int kLATENCY_ms = 100;
constexpr double kLATENCY_s = kLATENCY_ms/1000.0;

constexpr uint kPORT = 4567;
}

int main(int argc, char* argv[]) {
  uWS::Hub h;

  MPC mpc{};

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    cout << sdata << endl;

    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        const auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];

          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];

          // To deal with latency I adjust the state of a car to be as
          // predicted state in latency time in the future, I'm using
          // the same kinematic model as in MPC controller:
          const double delta = j[1]["steering_angle"];
          const double throttle = j[1]["throttle"];
          px += v*cos(psi)*kLATENCY_s;
          py += v*sin(psi)*kLATENCY_s;
          psi -= (v/kLF)*delta*kLATENCY_s;
          // throttle used as acceleration here is an approximation
          v += throttle*kLATENCY_s;

          // I preprocess waypoints (shift and rotate) so that car coordinate 
          // system aligns with global coordinate system
          for(size_t i = 0; i < ptsx.size(); ++i) {
            const auto shift_x = ptsx[i] - px;
            const auto shift_y = ptsy[i] - py;
            ptsx[i] = shift_x*cos(kREF_ANGLE-psi)-shift_y*sin(kREF_ANGLE-psi);
            ptsy[i] = shift_x*sin(kREF_ANGLE-psi)+shift_y*cos(kREF_ANGLE-psi);
          }

          Map<VectorXd> ptsx_trans(ptsx.data(), ptsx.size());
          Map<VectorXd> ptsy_trans(ptsy.data(), ptsy.size());

          // fit polynomial to transformed waypoints, the result are the
          // coefficients of reference line polynomial
          const auto coeffs = polyfit(ptsx_trans, ptsy_trans, kPOLY_ORDER);
          // cte and epsi calculation after taking into account that after
          // transformation car is at position (0,0) with psi 0:
          // cte = f(x_t) - y_t              <= yt is 0
          // epsi = psi_t - arctan(f'(x_t))  <= psi_t is 0, x_t is 0
          // f'(x_t) = coeffs[1] + 2*x_t*coeffs[2] + 3*x_t^2*coeffs[3]
          // after simplification (x_t=0)  f'(x_t) = coeffs[1]
          const auto cte = polyeval(coeffs, kREF_X);
          const auto epsi = -atan(coeffs[1]);

          /*
          * TODO: Calculate steering angle and throttle using MPC.
          * Both are in between [-1, 1].
          */

          VectorXd state(6);
          // after transformations car position is (0,0) with psi = 0
          state << kREF_X, kREF_Y, kREF_ANGLE, v, cte, epsi;

          // rune the solver
          const auto vars = mpc.Solve(state, coeffs);

          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals{};
          vector<double> mpc_y_vals{};

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line
          for(size_t i=2; i<vars.size(); ++i) {
            if(i%2 == 0) {
              mpc_x_vals.push_back(vars[i]);
            } 
            else {
              mpc_y_vals.push_back(vars[i]);
            }
          }

          //Display the waypoints/reference line
          constexpr size_t num_points = 10;
          constexpr double inc = 5;
          vector<double> next_x_vals(num_points);
          vector<double> next_y_vals(num_points);

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line
          for(size_t i=1; i <= num_points; ++i) {
            const auto next_x = i*inc;
            next_x_vals.push_back(next_x);
            next_y_vals.push_back(polyeval(coeffs, next_x));
          }

          json msgJson{};
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          
          // I multiplied the steering value by -1 to account for how 
          // simulator treats positive values (clockwise)
          msgJson["steering_angle"] = vars[0]/(deg2rad(25)*kLF) * -1.0;
          msgJson["throttle"] = vars[1];
          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          const auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(kLATENCY_ms));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } 
      else {
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
    cout << "Connected!!!" << endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    cout << "Disconnected" << endl;
  });

  if (h.listen(kPORT)) {
    cout << "Listening to port " << kPORT << endl;
  } else {
    std::cerr << "Failed to listen to port" << endl;
    return -1;
  }
  h.run();
}
