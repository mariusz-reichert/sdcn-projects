/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>
#include <cassert>

#include "particle_filter.hpp"

namespace particle_filter_project {

using namespace std;

namespace {

template <typename T>
string VecToString(const vector<T>& v) {
  stringstream ss{};
  copy(v.begin(), v.end(), ostream_iterator<T>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

template string VecToString<uint>(const vector<uint>& v);
template string VecToString<double>(const vector<double>& v);

/*
 * Computes the Euclidean distance between two 2D points.
 * @param (x1,y1) x and y coordinates of first point
 * @param (x2,y2) x and y coordinates of second point
 * @output Euclidean distance between two 2D points
 */
double CalcDistance(const double x1, const double y1, const double x2, const double y2) {
  return std::hypot(x2 - x1, y2 - y1);
}

}

void ParticleFilter::Init(const double x, const double y, const double theta) {
	// Initialize all particles to first position (based on estimates of 
	// x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.

  default_random_engine gen{}; 
  normal_distribution<double> N_x(x, kSIGMA_POS[0]);
  normal_distribution<double> N_y(y, kSIGMA_POS[1]);
  normal_distribution<double> N_theta(theta, kSIGMA_POS[2]);

  for (size_t i = 0; i < kNUM_PARTICLES; ++i) {
    Particle p{};
    p.id = i;
    p.x = N_x(gen);
    p.y = N_y(gen);
    p.theta = N_theta(gen);
    p.weight = 1.0;

    weights_.push_back(p.weight);
    particles_.push_back(move(p));
  }
}

void ParticleFilter::Predict(const double velocity, const double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.

  assert(weights_.size() > 0 and particles_.size() > 0);

  const double v_dt = velocity*kDELTA_T;
  const double v_over_yr = velocity/yaw_rate;
  const double yr_dt = yaw_rate*kDELTA_T;

  default_random_engine gen{};
  for (size_t i = 0; i < kNUM_PARTICLES; ++i) {

    double new_x{}, new_y{}, new_theta{};
    const double& theta = particles_[i].theta;
    const double arg = theta + yr_dt;
    const double sin_theta = sin(theta);
    const double cos_theta = cos(theta);

    if (fabs(yaw_rate) < 0.001) {
      new_x = particles_[i].x + v_dt*cos_theta;
      new_y = particles_[i].y + v_dt*sin_theta;
      new_theta = theta;
    }
    else {
      new_x = particles_[i].x + v_over_yr*(sin(arg) - sin_theta);
      new_y = particles_[i].y + v_over_yr*(cos_theta - cos(arg));
      new_theta = arg;
    }

    normal_distribution<double> N_x(new_x, kSIGMA_POS[0]);
    normal_distribution<double> N_y(new_y, kSIGMA_POS[1]);
    normal_distribution<double> N_theta(new_theta, kSIGMA_POS[2]);

    particles_[i].x = N_x(gen);
    particles_[i].y = N_y(gen);
    particles_[i].theta = N_theta(gen);
  }

}

void ParticleFilter::UpdateWeights(const vector<Landmark>& observations, const Map& map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 3.33
	//   http://planning.cs.uiuc.edu/node99.html

  constexpr auto sig_x = kSIGMA_LANDMARK[0];
  constexpr auto sig_y = kSIGMA_LANDMARK[1];
  constexpr auto var_x = sig_x*sig_x;
  constexpr auto var_y = sig_y*sig_y;
  constexpr auto normalizer = 2*M_PI*sig_x*sig_y;

  for (size_t p = 0; p < kNUM_PARTICLES; ++p) {
    vector<Landmark> tobs(observations.size()); // transformed observations
    particles_[p].weight = 1.0;
    const double cos_theta = cos(particles_[p].theta);
    const double sin_theta = sin(particles_[p].theta);

    for (size_t i = 0; i < observations.size(); ++i) {
      // perform transformation from vehicle to map coordinates
      tobs[i].x = particles_[p].x+(observations[i].x*cos_theta - observations[i].y*sin_theta);
      tobs[i].y = particles_[p].y+(observations[i].x*sin_theta + observations[i].y*cos_theta);
    }

    vector<uint> associations{};
    vector<double> sense_x{};
    vector<double> sense_y{};

    for (size_t i = 0; i < tobs.size(); ++i) {
      double smallest_dist_to_landmark = kSENSOR_RANGE;
      uint association = 0;

      for (size_t j = 0; j < map_landmarks.size(); ++j) {
        const double distance_to_landmark = CalcDistance(tobs[i].x, 
                                                         tobs[i].y, 
                                                         map_landmarks[j].x, 
                                                         map_landmarks[j].y);

        if (distance_to_landmark < smallest_dist_to_landmark) {
          smallest_dist_to_landmark = distance_to_landmark;
          association = j;
        }
      }

      const long double multiplier = exp(-(pow(tobs[i].x-map_landmarks[association].x, 2.0)
                                              / (2.0*var_x) 
                                           +
                                           pow(tobs[i].y-map_landmarks[association].y, 2.0)
                                              / (2.0*var_y))
                                        ) / normalizer;
      particles_[p].weight *= multiplier;

      associations.push_back(association+1);
      sense_x.push_back(tobs[i].x);
      sense_y.push_back(tobs[i].y);

      SetAssociations(particles_[p], associations, sense_x, sense_y);
      weights_[p] = particles_[p].weight;
    }
  }
}

void ParticleFilter::Resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
  default_random_engine gen{};
  discrete_distribution<int> distribution(weights_.begin(), weights_.end());
  vector<Particle> resampledParticles(particles_.size());

  for (size_t i = 0; i < kNUM_PARTICLES; ++i) {
    resampledParticles[i] = particles_[distribution(gen)];
  }
  particles_ = move(resampledParticles);
}

void ParticleFilter::SetAssociations(Particle& particle, vector<uint>& associations, vector<double>& sense_x, vector<double>& sense_y) {
	//particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
	// associations: The landmark id that goes along with each listed association
	// sense_x: the associations x mapping already converted to world coordinates
	// sense_y: the associations y mapping already converted to world coordinates

	//Clear the previous associations
	particle.associations.clear();
	particle.sense_x.clear();
	particle.sense_y.clear();

	particle.associations = associations;
 	particle.sense_x = sense_x;
 	particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(uint particle_idx) {
	return VecToString(particles_[particle_idx].associations);
}

string ParticleFilter::getSenseX(uint particle_idx) {
	return VecToString(particles_[particle_idx].sense_x);
}

string ParticleFilter::getSenseY(uint particle_idx) {
	return VecToString(particles_[particle_idx].sense_y);
}

} // namespace particle_filter_project
