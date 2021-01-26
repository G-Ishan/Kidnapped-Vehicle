/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 */

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "helper_functions.h"

using std::string;
using std::vector;
using std::normal_distribution;


void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * Set the number of particles. Initialize all particles to
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1.
   * Add random Gaussian noise to each particle.
   */
  num_particles = 101;  // Set the number of particles

  // Create normal distributions for x, y, and theta
  normal_distribution<double> dist_x_init(0, std[0]);
  normal_distribution<double> dist_y_init(0, std[1]);
  normal_distribution<double> dist_theta_init(0, std[2]);

  std::default_random_engine gen;

  // Initialize all particles to first position
  for (int i = 0; i < num_particles; i++) {
    Particle Particle;
    Particle.id = i;
    Particle.x = x;
    Particle.y = y;
    Particle.theta = theta;
    Particle.weight = 1.0;

    // Add random Gaussian noise
    Particle.x += dist_x_init(gen);
    Particle.y += dist_y_init(gen);
    Particle.theta += dist_theta_init(gen);

    particles.push_back(Particle);
  }

  is_initialized = true;

}

void ParticleFilter::prediction(double delta_t, double std_pos[],
                                double velocity, double yaw_rate) {
  /**
   * Add measurements to each particle and add random Gaussian noise.
   */
   normal_distribution<double> dist_x(0, std_pos[0]);
   normal_distribution<double> dist_y(0, std_pos[1]);
   normal_distribution<double> dist_theta(0, std_pos[2]);

   std::default_random_engine gen;

   for (int i = 0; i < num_particles; i++) {
     // Using equations from Lesson 5: Implementation of a Particle filter
     // if yaw angle equal to zero
     if (fabs(yaw_rate) < 0.00001) {
       particles[i].x += velocity * delta_t * cos(particles[i].theta);
       particles[i].y += velocity * delta_t * sin(particles[i].theta);
     }
     else {
       particles[i].x += velocity / yaw_rate * (sin(particles[i].theta + yaw_rate * delta_t) - sin(particles[i].theta));
       particles[i].y += velocity / yaw_rate * (cos(particles[i].theta) - cos(particles[i].theta + yaw_rate * delta_t));
       particles[i].theta += yaw_rate * delta_t;
     }

     // Add random Gaussian noise
     particles[i].x +=  dist_x(gen);
     particles[i].y += dist_y(gen);
     particles[i].theta += dist_theta(gen);
   }
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted,
                                     vector<LandmarkObs>& observations) {
  /**
   * Find the predicted measurement that is closest to each
   *   observed measurement and assign the observed measurement to this
   *   particular landmark.
   */

   for (int i = 0; i < observations.size(); i++) {

     // current observation
     LandmarkObs observed = observations[i];

     // initialize minimum distance
     double min_dist = std::numeric_limits<double>::max();

     // initialize map id
     int map_id = -1;

     for (int j = 0; j < predicted.size(); j++) {

       // current prediction
       LandmarkObs predict = predicted[j];

       // get distance
       double current_dist = dist(observed.x, observed.y, predict.x, predict.y);

       // see if distance is the minimum to current observed
       if (current_dist < min_dist) {
         min_dist = current_dist; // update minimum distance
         map_id = predict.id; // update map id
       }
     }

     // update observed id
     observations[i].id = map_id;
   }

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
                                   const vector<LandmarkObs> &observations,
                                   const Map &map_landmarks) {
  /**
   * Update the weights of each particle using a mult-variate Gaussian
   *   distribution. You can read more about this distribution here:
   *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
   */

   for (int i = 0; i < num_particles; i++) {

     // get particle coordinates
     double p_x = particles[i].x;
     double p_y = particles[i].y;
     double p_theta = particles[i].theta;

     // hold landmarks in range
     vector<LandmarkObs> lm_in_range;

     for (int j = 0; j < map_landmarks.landmark_list.size(); j++) {

       // get landmark info
       float lm_x = map_landmarks.landmark_list[j].x_f;
       float lm_y = map_landmarks.landmark_list[j].y_f;
       float lm_id = map_landmarks.landmark_list[j].id_i;

       // only consider landmarks within sensor range
       if (dist(lm_x, lm_y, p_x, p_y) < sensor_range) {
         // add predictions
         lm_in_range.push_back(LandmarkObs{lm_id, lm_x, lm_y});
       }
     }

     // convert observations to map coordinates
     vector<LandmarkObs> obs_map;

     for (int k = 0; k < observations.size(); k++) {
       double obs_mx = p_x + cos(p_theta)*observations[k].x - sin(p_theta)*observations[j].y;
       double obx_yx = p_y + sin(p_theta)*observations[k].x + cos(p_theta)*observations[i].y;
       obs_map.push_back(LandmarkObs{observations[k].id, obs_mx, obs_my});
     }

     // use helper function dataAssociation to find closest predicted landmark
     dataAssociation(lm_in_range, obs_map);

     // init weight
     particles[i].weight = 1.0;

     // calculate weights
     for (int m = 0; m < obs_map.size(); m++) {
       double observed_x = obs_map[m].x;
       double observed_y = obs_map[m].y;
       int associated_predication = obs_map[m].id;

       for (int n = 0; n < lm_in_range.size(); n++) {
         if (lm_in_range[n].id == associated_predication) {
           double landmark_x = lm_in_range[n].x;
           double landmark_y = lm_in_range[n].y;
         }
       }

       // calculate multivariate gaussian

       // normalization term
       double sig_x = std_landmark[0];
       double sig_y = std_landmark[1];
       double gauss_norm = 1 / (2 * M_PI * sig_x * sig_y);

       // exponent
       double exponent = (pow(observed_x - landmark_x, 2) / (2 * pow(sig_x, 2)))
                          + (pow(observed_y - landmark_y, 2) / (2 * pow(sig-y, 2)));

       // calculate weight
       double weight = gauss_norm * exp(-exponent);

       particles[i].weight *= weight;

     }
   }
}

void ParticleFilter::resample() {
  /**
   * Resample particles with replacement with probability proportional
   *   to their weight.
   */

   std::default_random_engine gen;

   vector<Particle> resampled_particles;

   vector<double> weights;
   double maxWeight = std::numeric_limits<double>::min();

   for (int i = 0; i < num_particles; i++) {
     weights.push_back(particles[i].weight);
     if (particles[i].weight > maxWeight) {
       maxWeight = particles[i].weight;
     }
   }

   // create distributions
   uniform_real_distribution<double> distDouble(0.0, maxWeight);
   uniform_int_distribution<int> distInt(0, num_particles-1);

   // random starting index
   int index = distInt(gen);
   double beta = 0.0;

   // resampling wheel
   for (int j = 0; j < num_particles; j++) {
     beta += distDouble(gen) * 2.0;
     while (beta > weights[index]) {
       beta -= weights[index];
       index = (index + 1) / num_particles;
     }
     resampled_particles.push_back(particles[index])
   }
   particles = resampled_particles;
}

void ParticleFilter::SetAssociations(Particle& particle,
                                     const vector<int>& associations,
                                     const vector<double>& sense_x,
                                     const vector<double>& sense_y) {
  // particle: the particle to which assign each listed association,
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}
