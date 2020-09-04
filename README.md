### Mapping of Tall Trees using UAV Mounted Instrumentation

#### Packages
* tree\_pointcloud\_viz
* tree\_mapping

#### Environment Variables
* Append gazebo environemnt variables accordingly (model path: ```GAZEBO\_MODEL\_PATH```, worlds path: ```GAZEBO\_RESOURCE\_PATH```)
* OR save models in ```~/.gazebo/models```.
* OR set path in ```package.xml``` as given here: ```<gazebo_ros gazebo_plugin_path="${prefix}/lib" gazebo_model_path="${prefix}/.." />```

Paths to set:
* Path to velodyne mode, velodyne plugins
* Path to tree model
* Path to Iris drone
