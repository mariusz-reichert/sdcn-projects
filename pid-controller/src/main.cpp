#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.hpp"
#include <math.h>

// for convenience
using json = nlohmann::json;

using namespace pid_controller_project;

namespace {

constexpr uint kPORT = 4567;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

}

int main(int argc, char* argv[])
{
  uWS::Hub h;

  /* REFLECTION
  I've choosen hyperparameters through manual tuning. I started first with
  proportional gain tuning and all others set to 0.0. Using Kp = -0.9 resulted
  in really big oscillations. So I tried much smaller value Kp = -0.1. Oscillations
  were reduced so then I kept Kp fixed to -0.1 and start tuning derivative gain.
  My first guess: Kd =-0.5 turned out to be ok. The car successfully drove
  around the track. With Kp and Kd having fixed values I started tuning Ki
  parameter. I tried values -0.1, -0.01. The first one really destroyed the
  controller. The car drove out of track after just a few seconds. The second
  was also wrong, the car drove only a little longer. Only reducing Ki by a factor 
  of 10 made the controller working again (Ki = -0.001). I decided to have just
  PD controller. I just felt that performance was better. My intutition is that: there
  is no systematic bias (error) in this system because after all it's just a simulator
  that has perfect accuracy/precision. So probably no reason to mitigate systematic bias
  using integral part of the controller.
  */

  constexpr double Kp = -0.1;
  constexpr double Ki = 0.0;
  constexpr double Kd = -0.5;
  PID pid{Kp, Ki, Kd};

  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      const auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        const auto j = json::parse(s);
        std::string event = j[0].get<std::string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          const double cte = std::stod(j[1]["cte"].get<std::string>());
          pid.UpdateError(cte);
          double steer_value = std::fmod(pid.TotalError(), 1.0);
          
          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
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

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  if (h.listen(kPORT))
  {
    std::cout << "Listening to port " << kPORT << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
