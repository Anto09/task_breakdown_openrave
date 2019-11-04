# task_breakdown_openrave
All code used for WPI MS Thesis and ICRA 2017

The project was for an NSF grant which looked to deploy low-cost robots in remote regions to assist with contaminated wearables removal.
The robots will have no visual sensing capabilities and limited computing resources, therefore they were to come shipped with pre-planned motions.
Operators were to input certain parameters such as:
  - Worker height and limb dimensions
  - Worker equipment
  - Worker strain levels
  to execute pre-planned trajectories.
  
The code included in this repository is all experimental and was used to conduct feasibility research for such a system.

Motion planning for the Baxter Robot was done with pre-planned motions executed on a pre-configured environment using OpenRAVE (http://openrave.org/)
The idea was to use approximated human dimensions (sizes of people) and highly accomodating motions to be able to quickly execute motions for most human types.
Hazmat equipment to be removed were also roughly modeled so as to allow for more precise planning.

This repository contains the following:
  1. Custom Rethink Baxter IKFAST (OpenRAVE) solvers
  2. Modified Rethink Baxter XML model configuration files
  3. Created Rethink Baxter and human DAE model configuration files (for OpenRave)
     - OpenRAVE did not have Baxter in its library of robots before this project
  3. Equipment to be removed as static robot models
  4. Meshes for collision detection
  5. Planning scripts and utilities
  6. ROS (https://www.ros.org/) scripts
  7. Test data and models 
  8. Backups
  
 This repository is not meant to be downloaded, built, and used, and only serves as the storage of all the files outlined above.
 
