A ROS package for interfacing and controlling the [GoPiGo3](https://www.dexterindustries.com/shop/gopigo-beginner-starter-kit/) differential drive robot.

# Dependencies
- [GoPiGo3](https://github.com/DexterInd/GoPiGo3)
- [differential-drive](https://github.com/thomasl86/differential-drive)

# Nodes:
- `distsensor`:
  - Published topics:
    - `distance` (std_msgs/Float64): Distance in [m]
  - Parameters:
    - `~rate` (*int* | default: 20 Hz)  
      Sampling rate
- `laserscanner`:
  - Published topics:
    - `laserscan` (sensor_msgs/LaserScan)  
      Single planar scan
    - `servo/cmd_pos` (std_msgs/Float64)  
      The commanded position of the servo onto which the laser distance sensor is mounted in [°].
  - Parameters:
    - `~dt_sweep` (*double* | default: 7.2 s)  
      Duration of a complete sweep
    - `~rate` (*int* | default: 20 Hz)  
      Sampling rate
    - `~do_sweep` (*boolean* | default: true)  
      Whether to carry out sweep or not
    - `~angle_sweep` (*double* | default: 180.0°)  
      The sweeping angle (e.g., 180.0° is from -90 to 90)
- `motors`:
  - Published topics:  
    - `lwheel` (std_msgs/Int16): The left wheel encoder count
    - `rwheel` (std_msgs/Int16): The the right wheel encoder count
  - Subscribed topics:  
    - `lmotor_cmd` (std_msgs/Float32): The commanded power going to the left wheel's motor (received from `differential_drive/pid_velocity`)
    - `rmotor_cmd` (std_msgs/Float32): The commanded power going to the right wheel's motor (received from `differential_drive/pid_velocity`)
  - Parameters:
    - `rate` (*int* | default: 30 Hz)  
      Sampling rate
- `servo`:  
  - Subscribed topics:  
    - `servo/cmd_pos` (std_msgs/Float64)  
      The commanded position of the servo in [°]; useful for when the servo is required to rotate at a defined rate (e.g. when used for laser-scanning)
  - Services:
    - `servo/set_pos`: Sets the angular position of the servo in [°]; range is [-90° 90°]
      - Arguments: angle [°]
  - Parameters:
    - `rate` (*int* | default: 30 Hz)  
      Sampling rate
    - `servo_id` (*int* | [1, 2] | default: 1):  
      The servo ID
