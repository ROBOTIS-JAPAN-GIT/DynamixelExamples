# DynamixelExamples

## Expected environment

### Directory hierarchy
The following environment is assumed.
```
catkin_ws/
    |- DynamixelSDK_jp_custom
    |- realsense-ros_jp_custom
    └- DynamixelExamples
```

## Examples

- `DynamixelPunTilt/`
- `other_example_to_be_released_later/`


### DynamixelPunTilt

- `puntilt_bringup` : is used for starting up a real machine.
- `puntilt_gazebo` : is used for simulating in gazebo.
- `puntilt_description` : defines 3D model for gazebo simulation or visualization.
- `puntilt_control` : is used for controlling puntilt.
  
#### usage in simulation

```
$ roslaunch puntilt_gazebo simulation_with_coke.launch
```

```
$ roslaunch puntilt_control tracking_target_color.launch
```

#### usage in real world

```
$ roslaunch puntilt_bringup {comming soon}
```

```
$ roslaunch puntilt_control tracking_target_color.launch
```
