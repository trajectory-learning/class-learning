
# Class-learning

task_learning_gh.m - the file with the classification based learning algorithm and the call of the optimization algorithm

cost_trajektorija_gh.m - the cost function for trajectory optimization

cmaes_v2.m - CMA-ES algorithm implementation


## Directories

demo1, demo2, demo3...demoN contain the input data for the algorithm, this data is gathered by the experimental setup in the teaching process

- script(1)(2)(3).txt - contain the demonstrated trajectories by kinesthetic
                        teaching, sampled from the UR robot
- TauN(1)(2)(3).txt - contain the object positions and orientations (X,Y,W-per row/object) before
                      kinesthetic teaching (initial configuration) captured by the Kinect sensor
 
 demoN/konfigN/
- scriptX.txt - contain the initial position of the UR robot for a new
                situation
- TauNX.txt - contains the initial position of the objects for a new
              situation (X,Y,W-per row/object)


## Dependencies

Robotics toolbox for Matlab
- http://petercorke.com/wordpress/toolboxes/robotics-toolbox,
- P.I. Corke, “Robotics, Vision & Control”, Springer 2017, ISBN 978-3-319-54413-7

