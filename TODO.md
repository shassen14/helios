# Things to think about

* Using glam instead of nalgebra
  * glam only goes up to 4x4 matrices but we can always make Vec<dvec3<f64>>
  * Will need to change custom integrators
  * This will most likely be helpful with the moment i change components to literal positions, velocity, etc.
  * Bevy also uses glam so it would probably be easier to transform coordinate systems, same math and also same types

* Use avian3d physics engine? maybe rapier?
  * The physics game engine will take care of collisions and any low level mathematics
  * I will use "high level" dynamic models and give the game engine items forces and torques to move the vehicles
  * For instance, longitudinal and lateral model for my high level and then create the rigid bodies with the physics engine
    * The forces calculated will be from the long and lat models applied to the bodies
    * Need to figure out the true positions, velocities, states, etc of the system itself, but that shouldn't be too bad?? 

* Obtaining wheelbases from any dynamic system if they have the value as well as maximum turn radius / steering change

* Path Planning has some issues
  * uses both a grid map and obstacle list to path plan.
  * This is for two trains of thought, one for grid planning, the other for continuous space
  * I do want the path planner to output a vector of states for both path planning and motion planning if available
  * Getting very complex in this regard...
  * Might need to use enums and states?? still complicated
  * also gotta worry about moving grids/maps for local planning, etc
  * local planning vs global planning

* UI things to take care of (Helpful for testing probably?)
  * Free view mode where I can move the camera around
  * Spawn obstacles and write to a file of the last saved obstacle course, and delete? 
  * Spawn anything tbh aka cars, planes, etc.
  * Plot certain conditions (ie. true state vs estimated state, control inputs, desired state vs estimated state, etc)

* Sensors
  * Inertial Sensors
    * IMU, accelerometers, gyroscopes, magnetometers, GPS, Altitude, 
  * Visual Sensors
    * Cameras, Radar, Lidar
    * ^ different types, stereo, mono, long ranger, FOV, 2D lidar, 3D lidar 
  * Add different types of noise, resolution, etc.

* Build algorithms for each system (sensor, sensor_suite, localization, mapping, global planner, local planner, controls, modeling) to test the pipeline "works"

* Multithread building each entity for faster loading since it takes a bit of time for single thread to build every entity
