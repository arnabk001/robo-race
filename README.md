# Final Report
### EE P 545 - The Self Driving Car
### Team 8: Arnab Karmakar, John Fuld, Rahul Vishwanath


## 1. Operating procedures


To run our system up from scratch, first power up and log into the robot, and place it in its starting pose. Using the computer that’s logged into the robot, run teleop.launch. With teleop.launch running, run PlannerNode.launch to load our predefined path. Next, run particlefilter.py to initialize localization. Then, using the 2d pose estimate tool in RViz, align the starting pose of the robot in the virtual environment with the starting pose of the physical robot. Finally, with the RB button on the robots controller pressed, run line_follower.py, and press enter when prompted. The robot should begin navigating the race course.



## 2. Key Modules description


Our team’s implementation leveraged work from previous pakages; specifically, our line follower implementation along with our particle filter and path planning implementations comprised the majority of our code base for robot control, localization and planner respectively.
 
Our planner defines a ROS node for our path planner designed to generate and execute plans for the robot to navigate from a source to a target. The node subscribes to source and target pose topics, and, upon receiving new poses, it plans a trajectory using the A* algorithm defined inside HaltonPlanner. The generated plan is then published, periodically updating the plan and checking for new source and target poses. 


For localization, a particle filter implementation is used for estimating the state of the robot, utilizing sensor and motion models to update and maintain a distribution of particles representing possible states of the robot. The code includes functionality for resampling, visualization, and initializing particles based on received initial poses. The main loop continuously checks for the need to resample and visualizes the current state. The implementation is integrated with the planner to update the estimated pose of the robot. 


Robot control is implemented using our line follower, which subscribes to the estimated pose of the robot, and using constant velocity and PID control of the steering angle, the robot is directed to follow the computed plan. 



## 3. Additional implementation


While working through our project, our team came up with a few tricks to optimize the performance of our robot for the final race. During initial testing, we found that the robot often got stuck in the virtual environment when navigating a precomputed path comprising good waypoints. As a solution, our team computed coordinates of additional waypoints for our robot to follow that would enable our robot to smoothly traverse the racecourse while avoiding bad waypoints, and added them to visualize_marker.py. During testing on the physical robot, we adjusted these waypoints further to ensure the robot reached all good waypoints.


Additionally, our team tuned several PID parameters during testing on the physical robot. Our team found that optimizing the PID parameters led to the robot following our precomputed path more closely, reducing effects of unpredictable mechanical perturbations in the robot that degraded performance.  



## 4. Result and discussion


Our team’s race results were near perfect; we were able to pass through 7 out of 8 good waypoints and did not run into any bad waypoints or boundaries during the race.


We had previously tested the robot hitting all good waypoints prior to the final race, but after charging our batteries, we ended up missing only one specific waypoint during the final race. In assignment 2, we had previously discerned that battery charge greatly affected robot performance, and since battery charge is the only parameter that changed between testing and our final race, it’s likely the key contributor to us missing a waypoint.


Github link for code: https://github.com/EE545/assignment-5-team-8 (this repo)


Note that our project also uses code from previous assignments. This code is available in the corresponding assignment repositories. 


Total path planned - showing waypoints
!["Screenshot_rospath.png"](https://github.com/EE545/assignment-5-team-8/blob/master/Screenshot_rospath.png)

Actual track race video (included in github)
