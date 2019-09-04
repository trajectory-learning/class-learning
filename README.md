
# Class-learning

task_learning_gh.m - it contains the classification based learning algorithm and the call of the optimization algorithm

cost_trajektorija_gh.m - the cost function for trajectory optimization

cmaes_v2.m - CMA-ES algorithm implementation (http://cma.gforge.inria.fr/cmaes_sourcecode_page.html)

attractor_point_dmp.m - DMP approximation 


## Directories

demo1, demo2, demo3...demoN - contain the input data for the algorithm (demonstrations), this data is gathered by the experimental setup in the teaching process

- script(1)(2)(3).txt - contain the demonstrated trajectories by kinesthetic
                        teaching, sampled from the UR robot
- TauN(1)(2)(3).txt - contain the object positions and orientations (X,Y,W-per row/object) before
                      kinesthetic teaching (initial configuration) captured by the Kinect sensor
        
 demoN/konfigN/ - contain different configurations of the robot workspace (object locations and robot initial position)
- scriptX.txt - contain the initial position of the UR robot for a new
                situation
- TauNX.txt - contains the initial position of the objects for a new
              situation (X,Y,W-per row/object)

## Examples

- demo1 - two position constraints
- demo2 - four position constraints with one obstacle
- demo3 - two position constraints with one obstacle
- demo4 - three position constraints
- demo5 - sweeping task, four position constraints one obstacle

## Usage

Run the task_learning_gh.m script with the specified *demo* variable, depending on which example wants to be run.
When optimization stops, run the attractor_points_dmp.m script to obtain the smooth robot trajectory. 


## Dependencies

Robotics toolbox for Matlab
- http://petercorke.com/wordpress/toolboxes/robotics-toolbox,
- P.I. Corke, “Robotics, Vision & Control”, Springer 2017, ISBN 978-3-319-54413-7

