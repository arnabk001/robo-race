#!/usr/bin/env python

import rospy
import numpy as np
from threading import Lock
import sys

if sys.version_info[0] == 2:
    range = xrange


'''
  Provides methods for re-sampling from a distribution represented by weighted samples
'''
class ReSampler:

  '''
    Initializes the resampler
    particles: The particles to sample from
    weights: The weights of each particle
    state_lock: Controls access to particles and weights
  '''
  def __init__(self, particles, weights, state_lock=None):
    self.particles = particles 
    self.weights = weights
    
    # For speed purposes, you may wish to add additional member variable(s) that 
    # cache computations that will be reused in the re-sampling functions
    # YOUR CODE HERE?
    
    if state_lock is None:
      self.state_lock = Lock()
    else:
      self.state_lock = state_lock
  
  '''
    Performs independently, identically distributed in-place sampling of particles
  '''
  def resample_naiive(self):
    self.state_lock.acquire()
    # YOUR CODE HERE
    
    num_particles = len(self.particles)
    
    # Create M bins, each with a width equal to the corresponding weight
    bins = np.cumsum(self.weights)

    # Draw M random numbers between 0.0 and 1.0
    # random_numbers = np.random.randn(num_particles)
    random_numbers = np.random.uniform(0.0, 1.0, num_particles)

    # Resample particles based on the bins and random numbers
    resampled_indices = np.digitize(random_numbers, bins)

    # Return the resampled particles
    resampled_particles = self.particles[resampled_indices]

    self.particles[:] = resampled_particles
    
    
    self.state_lock.release()
  
  '''
    Performs in-place, lower variance sampling of particles
    (As discussed in table 4.4 of Probabilistic Robotics)
  '''
  def resample_low_variance(self):
    self.state_lock.acquire()
    new_particles = []
    # YOUR CODE HERE
    num_particles = len(self.particles)
    r = np.random.uniform(0,1.0/num_particles)
    c = self.weights[0]
    i = 0
    for m in range(num_particles):
      u = r + np.float32(m)/np.float32(num_particles)
      while u > c:
        i += 1
        c = c + self.weights[i]
      new_particles.append(self.particles[i])

    self.particles[:] = np.array(new_particles) 
    
    self.state_lock.release()
    
import matplotlib.pyplot as plt

if __name__ == '__main__':

  rospy.init_node("sensor_model", anonymous=True) # Initialize the node

  n_particles = int(rospy.get_param("~n_particles",100)) # The number of particles    
  k_val = int(rospy.get_param("~k_val", 80)) # Number of particles that have non-zero weight
  resample_type = rospy.get_param("~resample_type", "naiive") # Whether to use naiive or low variance sampling
  trials = int(rospy.get_param("~trials", 10)) # The number of re-samplings to do
  
  histogram = np.zeros(n_particles, dtype=np.float32) # Keeps track of how many times
                                                    # each particle has been sampled
                                                    # across trials
  
  for i in range(trials):
    particles = np.repeat(np.arange(n_particles)[:,np.newaxis],3, axis=1) # Create a set of particles
                                                                          # Here their value encodes their index
    # Have increasing weights up until index k_val
    weights = np.arange(n_particles, dtype=np.float32)
    weights[k_val:] = 0.0
    weights[:] = weights[:] / np.sum(weights)
    
    rs = ReSampler(particles, weights) # Create the Resampler
  
    # Resample
    if resample_type == "naiive":
      rs.resample_naiive()
    elif resample_type == "low_variance":
      rs.resample_low_variance()
    else:
      print("Unrecognized resampling method: "+ resample_type)     

    # Add the number times each particle was sampled    
    for j in range(particles.shape[0]):
      histogram[particles[j,0]] = histogram[particles[j,0]] + 1
    
  # Display as histogram
  plt.bar(np.arange(n_particles), histogram)
  plt.xlabel('Particle Idx')
  plt.ylabel('# Of Times Sampled')
  plt.show()    

