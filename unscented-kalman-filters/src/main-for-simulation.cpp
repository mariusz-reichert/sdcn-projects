#include <uWS/uWS.h>
#include <iostream>
#include <memory>

#include "json.hpp"
#include "tools.hpp"
#include "measurement.hpp"
#include "state.hpp"
#include "sensor_fusion.hpp"

using namespace std;
using json = nlohmann::json;
using namespace unscented_kalman_filter_project;

namespace {

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string HasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos and b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

constexpr uint kPORT = 4567;

}

int main()
{
  uWS::Hub h;

  shared_ptr<SensorFusionUkf> filterHandle{};

  vector<Measurement> measurements{};
  vector<State> ground_truths{};
  vector<State> estimations{};

  h.onMessage([&](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event

    if (length and length > 2 and data[0] == '4' and data[1] == '2') {
      const auto s = HasData(string(data));
      
      if (s != "") {
        const auto j = json::parse(s);

        const string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          const string& sensor_measurment = j[1]["sensor_measurement"];
          istringstream iss{sensor_measurment};
          string sensor_type{};
          iss >> sensor_type;
         
          Measurement m{sensor_type == "L" ? SensorType::kLaser : SensorType::kRadar};
          iss >> m;
          
          State gt{};
          iss >> gt;
          iss.clear();

          if (not filterHandle) {
            filterHandle = make_shared<SensorFusionUkf>(m);
          }
          else {
            filterHandle->Process(m);
          }
          measurements.push_back(move(m));
          ground_truths.push_back(move(gt));

          const State& estimation = filterHandle->GetStateEstimation();
          estimations.push_back(estimation);

          const auto RMSE = tools::CalculateRmse(estimations, ground_truths);

          json msgJson{};
          msgJson["estimate_x"] = estimation(0);
          msgJson["estimate_y"] = estimation(1);
          msgJson["rmse_x"] =  RMSE(0);
          msgJson["rmse_y"] =  RMSE(1);
          msgJson["rmse_vx"] = RMSE(2);
          msgJson["rmse_vy"] = RMSE(3);
          auto msg = "42[\"estimate_marker\"," + msgJson.dump() + "]";
          cout << "Measured (x,y) = " << measurements.back() << endl;
          cout << "Estimated (x,y,vx,vy) = " << estimation << endl;
          cout << "RMSE (x,y,vx,vy) = " << RMSE << endl << endl;;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
	  
        }
      } 
      else {
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }

  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    }
    else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    cout << "Connected!!!" << endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    cout << "Disconnected" << endl;
  });

  if (h.listen(kPORT)) {
    cout << "Listening to port " << kPORT << endl;
  }
  else {
    cerr << "Failed to listen to port" << endl;
    return -1;
  }
  h.run();
}
