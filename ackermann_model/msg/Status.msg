Header header

# Velocity in World Frame Along Axis(m/s)
geometry_msgs/Twist velWorldFrame

# Accel in World Frame Along Axis(m/s^2)
geometry_msgs/Twist accelWorldFrame

# Velocity in Car Frame Along Axis(m/s)
geometry_msgs/Twist velCarFrame

# Accel in Car Frame Along Axis(m/s^2)
geometry_msgs/Twist accelCarFrame


# Throttle range 0 to 1, 1 is max Throttle
float32 throttle

# Brake range 0 to 1, 1 is max Brake
float32 brake

# Gear
int32 gear

# Steer range -1 to 1, 1 is maximum left turn
float32 steer


# Net Speed (m/s)
float64 net_speed

# Net Accel (m/s^2)
float64 net_acceleration