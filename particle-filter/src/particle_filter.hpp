#pragma once

#include <vector>
#include <string>

#include "map.hpp"

namespace particle_filter_project {

using std::vector;
using std::string;

constexpr uint kNUM_PARTICLES = 10;
constexpr double kDELTA_T = 0.1; // Time elapsed between measurements [sec]
constexpr double kSENSOR_RANGE = 10; // Sensor range [m]

// GPS measurement uncertainty [x [m], y [m], theta [rad]]:
constexpr double kSIGMA_POS[3] = {0.3, 0.3, 0.01}; 
// Landmark measurement uncertainty [x [m], y [m]]:
constexpr double kSIGMA_LANDMARK[2] = {0.3, 0.3}; 

struct Particle {
	int id;
	double x;
	double y;
	double theta;
	double weight;
	vector<uint> associations;
	vector<double> sense_x;
	vector<double> sense_y;
};

class ParticleFilter {
	// current particles
	vector<Particle> particles_;
	// weights of all particles
	vector<double> weights_;
	
	/*
	 * Set a particles list of associations, along with the associations calculated world x,y coordinates
	 */
	void SetAssociations(Particle& particle, vector<uint>& associations, vector<double>& sense_x, vector<double>& sense_y);

public:

	/**
	 * Init Initializes particle filter by initializing particles to Gaussian
	 *   distribution around first position and all the weights to 1.
	 * @param x Initial x position [m] (simulated estimate from GPS)
	 * @param y Initial y position [m]
	 * @param theta Initial orientation [rad]
	 */
	void Init(const double x, const double y, const double theta);

	/**
	 * Predict Predicts the state for the next time step using the process model.
	 * @param velocity Velocity of car from t to t+1 [m/s]
	 * @param yaw_rate Yaw rate of car from t to t+1 [rad/s]
	 */
	void Predict(const double velocity, const double yaw_rate);
	
	/**
	 * UpdateWeights Updates the weights for each particle based on the likelihood of the 
	 *   observed measurements. 
	 * @param observations Vector of landmark observations
	 * @param map Map contains map landmarks
	 */
	void UpdateWeights(const vector<Landmark>& observations, const Map& map_landmarks);
	
	/**
	 * Resample Resamples from the updated set of particles to form
	 *   the new set of particles.
	 */
	void Resample();

	const vector<Particle>& GetParticles() noexcept { return particles_; }
	string getAssociations(uint article_idx);
	string getSenseX(uint article_idx);
	string getSenseY(uint article_idx);
};


} //particle_filter_project
