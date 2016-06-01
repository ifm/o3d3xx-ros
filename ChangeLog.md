# Changes between o3d3xx-ros 0.2.4 and 0.2.5

* Addressed bug in unit tests (services test)

# Changes between o3d3xx-ros 0.2.3 and 0.2.4

* Implemented a new `/Trigger` service for cameras configured in
  software-triggered mode.
* Added a `assume_sw_trigger` parameter to the camera launch file to optimize
  some settings when running in software triggered mode.

# Changes between o3d3xx-ros 0.2.2 and 0.2.3

* Tweaks to support Kinetic Kame on Ubuntu 16.04 (Xenial)

# Changes between o3d3xx-ros 0.2.1 and 0.2.2

* Loosened up the libo3d3xx version checking in CMakeLists.txt

# Changes between o3d3xx-ros 0.2.0 and 0.2.1

### Frame id is configurable and topic names are relative

* See [PR17](https://github.com/lovepark/o3d3xx-ros/pull/17)

### libo3d3xx 0.4.0

* The new modularized `libo3d3xx` is now used as the underlying driver.

### test/test_services

* New test to ensure calling the `/Dump` and `/Config` services work.

### cmake/FindXXX

* Copied the following FindXXX cmake scripts from `libo3d3xx`:
  * Findo3d3xx\_camera
  * Findo3d3xx\_framegrabber
  * Findo3d3xx\_image

### Deprecations and Eliminations

* FileWriteNode has been removed from project. Use `rosbag`.

# Changes between o3d3xx-ros 0.1.8 and 0.2.0

### o3d3xx_node

* Aware of pluggable pcic schema masks
* Publishes unit vectors (on a latched topic)
* Publishes camera extrinsics
* Now that we are publishing a proper transform between the camera frame and
  the optical frame (see below) we are tagging our published topics with the
  appropriate frame_id. Specfically, the cartesian data are marked to be in the
  camera frame (using ROS conventions) and the other data are marked as in the
  optical frame (O3D conventions).

### launch/camera.launch

* You can now pass a schema mask parameter to the camera to selectively choose
  which images are streamed back from the camera to the host.

* Now that we are providing the ability to compute the Cartesian data off-board
  the camera, the tf2 static\_transform\_publisher is now publishing out a
  proper transform between the camera frame and the optical frame. If you
  compute the Cartesian data off-board the camera by utilizing the extrinsics,
  unit vectors, and radial distance image, those computed Cartesian values will
  be expressed in the camera optical frame. To get them into a standard ROS
  coord frame, the data will need to be transformed. By publishing this
  transform via tf2, users can use the tf2 API to compute the transformation.

### launch/o3d3xx.rviz

* Added an Axes marker to the display (not selected by default). This is
  necessary if you want to inspect the difference between the camera frame and
  its optical frame. Since there is no translation, only a rotation, flipping
  the coord on an axes marker seems easier to grok than how the tf display is
  rendering.

### cmake/Findlibo3d3xx.cmake

* Aware of new `libo3d3xx` default install location into `/usr`

### test/test_camera.py

* Added new unit test that tests:
  - Getting data from the camera
  - Computing the Cartesian data and comparing it to ground truth
  - To do the comparison to ground truther, the computed cartesian data
    must also be transformed, to do that we use the tf2 API and hence The
    transform from the optical frame to the camera frame that we are
    publishing is also tested.

NOTE: The unit test(s) currently require the hardware to be present as it works
with live data.

### Deprecations and Eliminations

* The amplitude histogram is no longer being published.
* The `rviz` config that colors the point cloud pixels with the x-depth has
  been eliminated. Pixels will be colored with the normalized amplitude.
* FileWriteNode has been deprecated. It is slated to disappear in the next
  minor release and definitely will be vaporized by 1.0.0. Data collection
  pipelines should be updated to use `rosbag` or some other tool.

# Changes between o3d3xx-ros 0.1.7 and 0.1.8

### o3d3xx_node

* A new parameter, `timeout_tolerance_secs` has been added. This is a timeout
  period to wait before attempting to restart the underlying frame grabber if
  it is currently timing out while asking for image data. This has been added
  to add a level of robustness to things like network cables getting yanked out
  or allowing the camera to undergo a power cycle without having to restart the
  ROS processes. Bascially, a way of autonomously recovering from anything that
  would break the TCP connection between the underlying ASIO event loop in
  `libo3d3xx` and the O3D camera's PCIC interface. Good for robots.

# Changes between o3d3xx-ros 0.1.6 and 0.1.7

### o3d3xx_node

* Publishes the raw amplitude image

### rviz

* The rviz configuration files now stack in the raw amplitude as a new pane
  along with the normalized amplitude and the amplitude histogram.

### launch/throttled.launch

* throttles the `raw_amplitude` topic

### file_writer_node

* The `raw_amplitude` image is now serialized to the file system as a
  (losslessly compressed) PNG.

# Changes between o3d3xx-ros 0.1.5 and 0.1.6

### launch/throttled.launch

* throttles the `xyz_image` topic

### launch/camera.launch

* Moved to tf2 for the static\_transform\_publisher. The transform from
  /o3d3xx/camera\_link to /o3d3xx/camera\_optical\_link is now published on
  /tf\_static (a latched topic) for efficiency.

### file_writer_node

* The `xyz_image` is now serialized to the file system in OpenCV FileStorage
  format (YAML).

# Changes between o3d3xx-ros 0.1.4 and 0.1.5

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

# Changes between o3d3xx-ros 0.1.3 and 0.1.4

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

# The initial release of o3d3xx-ros was 0.1.3
