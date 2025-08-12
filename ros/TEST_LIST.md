
# Basic setup
* Create your catkin workspace 
* Go to `catkin_ws/src` and symlink the ros package
    * `ln -s /path/to/ros/ros`
* Build (from `catkin_ws`)
    `catkin_make`

# Can get points in the subscriber
* Launch the manager
* Change the pcap file in the config yml to some local pcap
* Launch the publisher
* Launch the subscriber

<strong>Should see `Got XXX points` messages</strong>


# Can see points in rviz
* Launch rviz
* Load the provided rviz config file
* Should see points

# Check available topics
* Run `rostopic list`

```
/cepton3/cepton_sensor_status
/cepton3/info_handle_3276805
/cepton3/points
/cepton3/points_handle_10994053554208
/cepton3/points_sn_3276805
/cepton3/sensor_information
/cepton_manager/bond
/rosout
/rosout_agg
```

# Checking sensor information
* Run `rostopic echo cepton3/sensor_information`


# Check the point filters
* Change the `max_azimuth` to 0. 
* On reloading the rviz config, you should only see points to the right of the +X axis

* Change the `min_altitude` to 0.
* On reloading the rviz config, you should only see points from the top half of the FOV

