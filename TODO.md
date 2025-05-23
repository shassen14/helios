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
