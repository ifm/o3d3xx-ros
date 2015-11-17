## Changes between o3d3xx-ros 0.1.7 and 0.1.8

## o3d3xx_node

* A new parameter, `timeout_tolerance_secs` has been added. This is a timeout
  period to wait before attempting to restart the underlying frame grabber if
  it is currently timing out while asking for image data. This has been added
  to add a level of robustness to things like network cables getting yanked out
  or allowing the camera to undergo a power cycle without having to restart the
  ROS processes. Bascially, a way of autonomously from anything that would
  break the TCP connection between the underlying ASIO event loop in
  `libo3d3xx` and the O3D camera's PCIC interface. Good for robots.

## Changes between o3d3xx-ros 0.1.6 and 0.1.7

## o3d3xx_node

* Publishes the raw amplitude image

## rviz

* The rviz configuration files now stack in the raw amplitude as a new pane
  along with the normalized amplitude and the amplitude histogram.

## launch/throttled.launch

* throttles the `raw_amplitude` topic

## file_writer_node

* The `raw_amplitude` image is now serialized to the file system as a
  (losslessly compressed) PNG.

## Changes between o3d3xx-ros 0.1.5 and 0.1.6

### launch/throttled.launch

* throttles the `xyz_image` topic

### launch/camera.launch

* Moved to tf2 for the static\_transform\_publisher. The transform from
  /o3d3xx/camera\_link to /o3d3xx/camera\_optical\_link is now published on
  /tf\_static (a latched topic) for efficiency.

### file_writer_node

* The `xyz_image` is now serialized to the file system in OpenCV FileStorage
  format (YAML).

## Changes between o3d3xx-ros 0.1.4 and 0.1.5

### o3d3xx_node

* The `xyzi_image` has been replaced by an `xyz_image`. The `xyz_image` is a
  representation of the point cloud encoded as an opencv image (CV_16SC3) where
  the 3 image planes are: 0 = x, 1 = y, 2 = z. The data in this image are
  signed 16-bit integers and the units are in mm. This is a change from the
  original implementation of the `xyzi_image` introduced in version 0.1.4. The
  intensity data can still be referenced via the `amplitude` image. Keeping the
  units in mm allows for using the more efficient 16-bit signed int type as
  opposed to 32-bit floats. Also note that since the image is being constructed
  by the underlying `libo3d3xx` driver, there is no longer a flag in the launch
  file to turn on/off the publishing of this data.

### launch/camera.launch

* Fixed the parent-child relationship between the tf frames

## Changes between o3d3xx-ros 0.1.3 and 0.1.4

### o3d3xx_node

* All published topics now have timestamps in the message header
* The main camera node can now publish an `xyzi_image`. This is a
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
