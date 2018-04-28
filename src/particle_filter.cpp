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

    num_particles = 10;
	// This line creates a normal (Gaussian) distribution for x,y,theta
	normal_distribution<double> dist_x(x, std[0]);
	normal_distribution<double> dist_y(y, std[1]);
	normal_distribution<double> dist_theta(theta, std[2]);
	
	particles.resize(num_particles);
	default_random_engine gen;
	for (int i = 0; i < num_particles; ++i) {
    	particles[i].id = i;
    	particles[i].x = dist_x(gen);
    	particles[i].y = dist_y(gen);
    	particles[i].theta = dist_theta(gen);
    	particles[i].weight = 1.;
	}
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
	
	
	default_random_engine gen;

    //predict next t+1 car position for each particle according to control input	
	for (int i = 0; i < num_particles; ++i) {
        double x0 = particles[i].x;
        double y0 = particles[i].y;
        double theta0 = particles[i].theta;
        
        double xf = 0., yf = 0., thetaf = 0.;
        if(fabs(yaw_rate) < 1e-3){
            xf += x0 + velocity * delta_t * cos(theta0);
            yf += y0 + velocity * delta_t * sin(theta0);        
        }
        else{        
	        xf = x0 + velocity/yaw_rate * (sin(theta0 + yaw_rate * delta_t) - sin(theta0));
	        yf = y0 + velocity/yaw_rate * (cos(theta0) - cos(theta0 + yaw_rate * delta_t));      
	        thetaf = theta0 + yaw_rate * delta_t;
	    }
	    	    	    	    
	    normal_distribution<double> dist_x(xf, std_pos[0]);
    	normal_distribution<double> dist_y(yf, std_pos[1]);
	    normal_distribution<double> dist_theta(thetaf, std_pos[2]);	    
	    
	    //add noise
	    particles[i].x = dist_x(gen);
	    particles[i].y = dist_y(gen);
	    particles[i].theta = dist_theta(gen);
	    
	    	    	    	    
	    
	}
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
	
	    
      for(int i = 0; i < observations.size();++i){
            
        double x_measured_transformed_obs = observations[i].x;
        double y_measured_transformed_obs = observations[i].y;
        
        double min_dist = 0.;
        //cout<<"----------------obs "<<i<<"-----------------"<<endl;
        //find nearets landmark for the observation from observable landmarks
        for(int j = 0; j < predicted.size();++j){
            double x_map_landmark = predicted[j].x;
            double y_map_landmark = predicted[j].y;                                
            double distance = dist(x_map_landmark, y_map_landmark, x_measured_transformed_obs, y_measured_transformed_obs);
            //cout<<j<<" assoc: ("<<x_measured_transformed_obs<<","<<y_measured_transformed_obs<<")"<<"("<<x_map_landmark<<","<<y_map_landmark<<") dist:"<<distance<<endl;
                                            
            if(j == 0 || min_dist > distance){
                min_dist = distance;
                observations[i].id = j;
            }
        }      
        //cout<<observations[i].id<<" best: ("<<x_measured_transformed_obs<<","<<y_measured_transformed_obs<<")"<<
        //"("<<predicted[observations[i].id].x<<","<<predicted[observations[i].id].y<<") dist:"<<min_dist<<endl;

    }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html
		
	for (int k = 0; k < num_particles; ++k) {
        double x_p = particles[k].x;
        double y_p = particles[k].y;
        double theta_p = particles[k].theta;
        particles[k].associations.clear();
	    particles[k].sense_x.clear();
	    particles[k].sense_y.clear();
	    
       	//Prepare a set of landmarks observable according to car position and sensor range	
        std::vector<LandmarkObs> observable_map_landmarks;
        for(int j = 0;j < map_landmarks.landmark_list.size(); ++j){
            double x_l = map_landmarks.landmark_list[j].x_f;
            double y_l = map_landmarks.landmark_list[j].y_f;

            if(dist(x_p, y_p, x_l, y_l) <= sensor_range){
                LandmarkObs landmark;
                landmark.x = x_l;
                landmark.y = y_l;    
                landmark.id = map_landmarks.landmark_list[j].id_i;
                observable_map_landmarks.push_back(landmark);
            }
        }
        //cout<<"==============particle=============="<<k<<endl;
        //cout<<"observable landmarks:"<<observable_map_landmarks.size()<<" of "<<map_landmarks.landmark_list.size()<<endl;
        
        //transform measurement observations to map coordinates        
        std::vector<LandmarkObs> transformed_observations;
        transformed_observations.resize(observations.size());
        for(int j = 0; j < observations.size();++j){
            double x_measured_obs = observations[j].x;
            double y_measured_obs = observations[j].y;
            
            transformed_observations[j].x = x_p + cos(theta_p)*x_measured_obs - sin(theta_p)*y_measured_obs;
	        transformed_observations[j].y = y_p + sin(theta_p)*x_measured_obs + cos(theta_p)*y_measured_obs;
        }
                
        //associate landmarks and observations
        dataAssociation(observable_map_landmarks, transformed_observations);
        
        double total_weight = 1.;
        //calculate density probability for each pair landmark-observation (multivariate gaussian probability)
        for(int j = 0; j < transformed_observations.size(); ++j){
            int assoc_map_landmark_idx = transformed_observations[j].id;
            LandmarkObs map_landmark = observable_map_landmarks[assoc_map_landmark_idx];
                        
            double sig_x = std_landmark[0];
            double sig_y = std_landmark[1];
            double x_obs = transformed_observations[j].x;
            double y_obs = transformed_observations[j].y;
            double mu_x = map_landmark.x;
            double mu_y = map_landmark.y;

            //calculate normalization term
            double gauss_norm = (1./(2. * M_PI * sig_x * sig_y));
            //calculate exponent
            double exponent = pow(x_obs - mu_x, 2.)/(2 * pow(sig_x,2)) + pow(y_obs - mu_y, 2.)/(2 * pow(sig_y,2.));
            double prob_density = gauss_norm * exp(-exponent);
            //cout<<"density for ("<<x_obs<<","<<y_obs<<")-("<<mu_x<<","<<mu_y<<")="<<prob_density<<endl;

            //calculate total weight by multiplying particular density probability
            total_weight*=prob_density;
            
            particles[k].associations.push_back(observable_map_landmarks[assoc_map_landmark_idx].id);
    	    particles[k].sense_x.push_back(x_obs);
    	    particles[k].sense_y.push_back(y_obs);            
        }
        particles[k].weight = total_weight;
	}
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

     std::vector<double> probabilities;
     probabilities.resize(particles.size());
     std::vector<Particle> resampled_particles;
     resampled_particles.resize(particles.size());
     
     //put all probabilities to array
     for(int i=0;i<particles.size();++i){
         probabilities[i] = particles[i].weight;
     }
     
     //make a radnom discrete distribution from probabilities array
     std::default_random_engine gen;
     std::discrete_distribution<> distribution(probabilities.begin(), probabilities.end());
     for(int i=0;i<particles.size();++i){
        //indexes will be generated according to the probabilities array
        //so we randomly choose particles according to their probabilities
        int idx = distribution(gen);
        resampled_particles[i] = particles[idx];
     }
     particles = resampled_particles;     
            
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
