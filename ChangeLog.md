## Changes between o3d3xx-ros 0.1.3 and 0.1.4

### o3d3xx_node

* All published topics now have timestamps in the message header
* The main camera node can now publish a `numpy_cloud`. This is a
  representation of the point cloud (same exact data) encoded as an opencv
  image (CV_32FC4) where the 4 image planes are: 0 = x, 1 = y, 2 = z, 3 =
  intensity. This has been introduced to enable easier interop between PCL and
  numpy for doing point cloud analysis. While you can iterate over point
  cloud messages in python with existing ROS-provided tools. It is dreadfully
  slow. By providing an OpenCV image encoding of the point cloud, a numpy
  array can be created at C++ speed. Publishing this image can be turned on/off
  via the `camera.launch` file so that no additional overhead will be realized
  by users who do not wish to exploit this feature.

### file_writer_node

* A dump of the camera configuration (JSON) is now written to the output
  directory upon launching the node, prior to data capture.

## The initial release of o3d3xx-ros was 0.1.3
