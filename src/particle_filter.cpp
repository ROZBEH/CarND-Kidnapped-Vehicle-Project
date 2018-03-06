/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
	num_particles = 1000;
	default_random_engine gen;
	normal_distribution<double> dist_x(gps_x, std[0]);
	normal_distribution<double> dist_y(gps_y, std[1]);
	normal_distribution<double> dist_theta(theta, std[2]);
	for (vector<Particle>::iterator part=Particles.begin(); part!=Particles.end(); ++t) 
    {
    	part.x = dist_x(gen);
    	part.y = dist_y(gen);
    	part.theta = dist_theta(gen);
    	part.weight = 1.0;
    }

	is_initialized = true;


}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
	default_random_engine gen;
	for (vector<Particle>::iterator part=Particles.begin(); part!=Particles.end(); ++t) 
    {
    	normal_distribution<double> dist_x(part.x, std_pos[0]);
		normal_distribution<double> dist_y(part.y, std_pos[1]);
		normal_distribution<double> dist_theta(part.theta, std_pos[2]);
		if (yaw_rate == 0){
			part.x = dist_x(gen) + velocity * delta_t*cos(part.theta);
			part.y = dist_y(gen) + velocity * delta_t*sin(part.theta);
			part.theta = dist_theta(gen);
		} else{
    	part.x = dist_x(gen) + (velocity/yaw_rate)*(sin(part.theta+yaw_rate*delta_t)-sin(part.theta));
    	part.y = dist_y(gen) + (velocity/yaw_rate)*(cos(part.theta)-cos(part.theta+yaw_rate*delta_t));
    	part.theta = dist_theta(gen) + yaw_rate*delta_t;
    		}
    }

}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
	// compare it with others and returns the ID
	vector<LandmarkObs>::iterator begin2 = predicted.begin();
  	vector<LandmarkObs>::iterator begin1 = observations.begin();
  	vector<LandmarkObs>::iterator end2 = predicted.end();
  	vector<LandmarkObs>::iterator end1 = observations.end();
  	vector<LandmarkObs>::iterator i1;
  	vector<LandmarkObs>::iterator i2;
  	for (i2 = begin2; i2 != end2; ++i2) {
  		double min_dist = numeric_limits< double >::max();
  		for (i1 = begin1; i1 != end1; ++i1) {
  			double distance = dist(double i2.x, double i2.y, double i1.x, double i1.y);
  			if ( distance < min_dist){
  				min_dist = distance;
  				// a poniter that points to the element that I found
  				LandmarkObs * pred = &(*i1);
  			}
  		}
  		// assign the closest prediction to i2
  		&(*i2) = pred;

  }

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	// more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	// according to the MAP'S coordinate system. You will need to transform between the two systems.
	// Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	// The following is a good resource for the theory:
	// https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	// and the following is a good resource for the actual equation to implement (look at equation 
	// 3.33
	// http://planning.cs.uiuc.edu/node99.html
	vector<LandmarkObs> trans_points;
	vector<LandmarkObs>::iterator obs_ = observations.begin();
  	vector<LandmarkObs>::iterator map_ = map_landmarks.begin();
  	for (; obs_!=observations.end() && map_!=map_landmarks.end(); ++obs_, ++map_)
  	{
  		float x_ = obs_.x * cos(map_.theta) - obs_.y * sin(map_.theta) + map_.x_f;
  		float y_ = obs_.x * sin(map_.theta) + obs_.y * cos(map_.theta) + map_.y_f
  		trans_points.push_back ({x_, y_});
  	}
  	// Now we want to see which map landmark is the closest to the trans_points
  	ParticleFilter::dataAssociation(trans_points, map_landmarks)
  	// Now we pass it to the Gaussian function to find out the overall probability
  	

}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
