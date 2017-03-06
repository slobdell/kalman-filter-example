This is as straightforward of an example as possible of sensor fusion between a GPS and an accelerometer using a kalman filter.

To use:

```
go run kalmanFilter.go
```

For corresponding video, visit:
[YouTube Video](https://youtu.be/jBynioLv2Es)

The above file is some sample data using a GPS and an accelerometer. Previous work extracted out gravity, and resultant quaternion from gyroscope and magnetomer was used to create readings for absolute acceleration in North, East, and Up.  How to ascertain those values is outside the scope of this project, but if you'd like help with that feel free to contact me.

The code itself is an API to fuse accelerometer and GPS data together in an extremely common scenario for using a kalman filter. The `taco_bell_data.json` is the input file, and an output file is produced that includes the estimated velocity and position at each sample without the aid of GPS.

There are additional helper functions in the file to translate GPS data to meters.
