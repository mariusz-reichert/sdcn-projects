#include <uWS/uWS.h>
#include <iostream>

#include "json.hpp"
#include "particle_filter.hpp"
#include "tools.hpp"

using namespace particle_filter_project;

namespace {

constexpr uint kPORT = 4567;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
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

}


int main(int argc, char* argv[]) {
  using namespace std;

  uWS::Hub h;

  Map map{};
  tools::ReadMapData("../data/map_data.txt", map);

  ParticleFilter pf{};
  bool initialized{false};

  h.onMessage([&map,&pf,&initialized](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length and length > 2 and data[0] == '4' and data[1] == '2') {
      auto s = hasData(string(data));

      if (s != "") {
        auto j = nlohmann::json::parse(s);
        const string event = j[0].get<string>();
        
        if (event == "telemetry") {
          if (not initialized) {
          	// Sense noisy position data from the simulator
            const double sense_x = stod(j[1]["sense_x"].get<string>());
			      const double sense_y = stod(j[1]["sense_y"].get<string>());
			      const double sense_theta = stod(j[1]["sense_theta"].get<string>());

			      pf.Init(sense_x, sense_y, sense_theta);
            initialized = true;
		      }
		      else {
            // Predict the vehicle's next state from previous (noiseless control) data.
		        const double previous_velocity = stod(j[1]["previous_velocity"].get<string>());
			      const double previous_yawrate = stod(j[1]["previous_yawrate"].get<string>());

			      pf.Predict(previous_velocity, previous_yawrate);
		      }

		      // receive noisy observation data from the simulator
		      // sense_observations in JSON format [{obs_x,obs_y},{obs_x,obs_y},...{obs_x,obs_y}]
          string sense_observations_x = j[1]["sense_observations_x"];
          string sense_observations_y = j[1]["sense_observations_y"];
          istringstream iss_x(sense_observations_x);
          istringstream iss_y(sense_observations_y);

		      vector<Landmark> noisy_observations{};
		      vector<double> x_sense{};
          vector<double> y_sense{};
          copy(istream_iterator<double>(iss_x),
               istream_iterator<double>(),
               back_inserter(x_sense));
  		    copy(istream_iterator<double>(iss_y),
            	 istream_iterator<double>(),
            	 back_inserter(y_sense));

          for(size_t i = 0; i < x_sense.size(); ++i){
            Landmark landmark_observation{};
            landmark_observation.x = x_sense[i];
			   	  landmark_observation.y = y_sense[i];
			   	  noisy_observations.push_back(move(landmark_observation));
          }

		      pf.UpdateWeights(noisy_observations, map);
		      pf.Resample();

		      // Calculate and output the average weighted error of the particle filter over all time steps so far.
		      double highest_weight = -1.0;
		      uint best_particle_idx = 0;
		      double weight_sum = 0.0;

		      for (uint i = 0; i < pf.GetParticles().size(); ++i) {
			      if (pf.GetParticles()[i].weight > highest_weight) {
			   	    highest_weight = pf.GetParticles()[i].weight;
			   	    best_particle_idx = i;
			      }
			      weight_sum += pf.GetParticles()[i].weight;
		      }

		      cout << "highest weight = " << highest_weight << endl;
		      cout << "average weight = " << weight_sum/kNUM_PARTICLES << endl << endl;

          nlohmann::json jsonMsg;
          jsonMsg["best_particle_x"] = pf.GetParticles()[best_particle_idx].x;
          jsonMsg["best_particle_y"] = pf.GetParticles()[best_particle_idx].y;
          jsonMsg["best_particle_theta"] = pf.GetParticles()[best_particle_idx].theta;

          //Optional message data used for debugging particle's sensing and associations
          jsonMsg["best_particle_associations"] = pf.getAssociations(best_particle_idx);
          jsonMsg["best_particle_sense_x"] = pf.getSenseX(best_particle_idx);
          jsonMsg["best_particle_sense_y"] = pf.getSenseY(best_particle_idx);

          const auto msg = "42[\"best_particle\"," + jsonMsg.dump() + "]";
          // cout << msg << endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
	  
        }
      } else {
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
    exit(EXIT_FAILURE);
  }

  h.run();
}























































































