o3d3xx-ros
==========
o3d3xx-ros is a wrapper around
[libo3d3xx](https://github.com/lovepark/libo3d3xx) enabling the usage of IFM
Efector O3D3xx ToF cameras from within [ROS](http://ros.org) software
systems.

Software Compatibility Matrix
-----------------------------
<table>
         <tr>
             <th>o3d3xx-ros version</th>
             <th>libo3d3xx version</th>
             <th>ROS distribution(s)</th>
         </tr>
         <tr>
             <td>0.1.3</td>
             <td>0.1.9</td>
             <td>Indigo</td>
         </tr>
         <tr>
             <td>0.1.4</td>
             <td>0.1.9</td>
             <td>Indigo</td>
         </tr>
         <tr>
             <td>0.1.5</td>
             <td>0.1.11</td>
             <td>Indigo</td>
         </tr>
         <tr>
             <td>0.1.6</td>
             <td>0.1.11</td>
             <td>Indigo</td>
         </tr>
         <tr>
             <td>0.1.7</td>
             <td>0.2.0</td>
             <td>Indigo</td>
         </tr>
         <tr>
             <td>0.1.8</td>
             <td>0.2.0</td>
             <td>Indigo</td>
         </tr>
         <tr>
             <td>0.2.0</td>
             <td>0.3.0</td>
             <td>Indigo</td>
         </tr>
         <tr>
             <td>0.2.1</td>
             <td>0.4.0</td>
             <td>Indigo</td>
         </tr>
         <tr>
             <td>0.2.2</td>
             <td>0.4.0, 0.4.1, 0.4.2</td>
             <td>Indigo</td>
        </tr>
        <tr>
             <td>0.2.3</td>
             <td>0.4.3</td>
             <td>Indigo, Kinetic</td>
        </tr>
        <tr>
             <td>0.2.4</td>
             <td>0.4.4</td>
             <td>Indigo, Kinetic</td>
        </tr>
        <tr>
             <td>0.2.5</td>
             <td>0.4.5, 0.4.6, 0.4.8, 0.4.9, 0.5.0, 0.6.0, 0.7.0</td>
             <td>Indigo, Kinetic</td>
        </tr>
</table>

Prerequisites
-------------

1. [Ubuntu 14.04 or 16.04](http://www.ubuntu.com)
2. [ROS Indigo or Kinetic](http://www.ros.org/install)
3. [libo3d3xx](https://github.com/lovepark/libo3d3xx)

Additionally, your compiler must support C++11. This package has been validated
with:

* g++ 4.8.2 on Ubuntu 14.04 LTS, ROS Indigo
* g++ 5.3.x on Ubuntu 16.04 LTS, ROS Kinetic

Building and Installing the Software
------------------------------------

__NOTE__: Since we are talking about ROS here, we assume you are on Ubuntu
Linux.

You should first ensure that you have installed ROS by following
[these](http://wiki.ros.org/ROS/Installation) instructions. The
`desktop-full` installation is highly recommended.

Next, you should be sure to install
[libo3d3xx](https://github.com/lovepark/libo3d3xx). This ROS package assumes
you have installed libo3d3xx via the supported debian installer. Step-by-step
instructions for that process now follows:

    $ git clone https://github.com/lovepark/libo3d3xx.git
    $ cd libo3d3xx
    $ mkdir build
    $ cd build
    $ cmake ..
    $ make

We now move on to building o3d3xx-ros.

Building and installing o3d3xx-ros is accomplished by utilizing the ROS
[catkin](http://wiki.ros.org/catkin) tool. There are many tutorials and other
pieces of advice available on-line advising how to most effectively utilize
catkin. However, the basic idea is to provide a clean separation between your
source code repository and your build and runtime environments. The
instructions that now follow represent how we choose to use catkin to build and
_permanently install_ a ROS package from source.

First, we need to decide where we want our software to ultimately be
installed. For purposes of this document, we will assume that we will install
our ROS packages at `~/ros`. For convenience, we add the following to our
`~/.bash_profile`:

    if [ -f /opt/ros/indigo/setup.bash ]; then
        source /opt/ros/indigo/setup.bash
    fi

    cd ${HOME}

    export LPR_ROS=${HOME}/ros

    if [ -d ${LPR_ROS} ]; then
        for i in $(ls ${LPR_ROS}); do
            if [ -d ${LPR_ROS}/${i} ]; then
                if [ -f ${LPR_ROS}/${i}/setup.bash ]; then
                    source ${LPR_ROS}/${i}/setup.bash --extend
                fi
            fi
        done
    fi

Next, we need to get the code from github. We assume we keep all of our git
repositories in `~/dev`.

    $ cd ~/dev
    $ git clone https://github.com/lovepark/o3d3xx-ros.git

We now have the code in `~/dev/o3d3xx-ros`. Next, we want to create a _catkin
workspace_ that we can use to build and install that code from. It is the
catkin philosophy that we do not do this directly in the source directory.

    $ cd ~/dev
    $ mkdir o3d3xx-catkin
    $ cd o3d3xx-catkin
    $ mkdir src
    $ cd src
    $ catkin_init_workspace
    $ ln -s ~/dev/o3d3xx-ros o3d3xx

So, you should have a catkin workspace set up to build the o3d3xx-ros code that
looks basically like:

    [ ~/dev/o3d3xx-catkin/src ]
    tpanzarella@jelly: $ pwd
    /home/tpanzarella/dev/o3d3xx-catkin/src

    [ ~/dev/o3d3xx-catkin/src ]
    tpanzarella@jelly: $ ls -l
    total 0
    lrwxrwxrwx 1 tpanzarella tpanzarella 49 Dec  2 15:26 CMakeLists.txt -> /opt/ros/indigo/share/catkin/cmake/toplevel.cmake
    lrwxrwxrwx 1 tpanzarella tpanzarella 32 Dec  2 15:24 o3d3xx -> /home/tpanzarella/dev/o3d3xx-ros

Now we are ready to build the code.

    $ cd ~/dev/o3d3xx-catkin
    $ catkin_make -DCATKIN_ENABLE_TESTING=ON
    $ catkin_make run_tests
    $ catkin_make -DCMAKE_INSTALL_PREFIX=${LPR_ROS}/o3d3xx install

The ROS package should now be installed in `~/ros/o3d3xx`. To test everything
out you should open a fresh bash shell, and start up a ROS core:

    $ roscore

Open another shell and start the primary camera node:

    $ roslaunch o3d3xx camera.launch ip:=192.168.10.69

__NOTE__: The IP address of your camera may differ. If you are using the
factory default (192.168.0.69), you do not need to specify it on the above
`roslaunch` line.

Open another shell and start the rviz node to visualize the data coming from
the camera:

    $ roslaunch o3d3xx rviz.launch

At this point, you should see an rviz window that looks something like:

![rviz1](doc/figures/rviz1.png)

Congratulations! You can now utilize o3d3xx-ros.

Nodes
-----

### /o3d3xx/camera

This node provides a real-time feed to the camera data. This node is started
from the primary `camera.launch` file:

    $ roslaunch o3d3xx camera.launch

The naming of the camera can be customized via the `ns` (namespace) and `nn`
(node name) command line arguments passed to the `camera.launch` file. For
example, if you specify your roslaunch command as:

    $ roslaunch o3d3xx camera.launch ns:=robot nn:=front_camera

The node will have the name `/robot/front_camera` in the ROS computation
graph.

#### Published Topics
<table>
         <tr>
             <th>Topic</th>
             <th>Message</th>
             <th>Description</th>
         </tr>

         <tr>
             <td>/o3d3xx/camera/amplitude</td>
             <td>sensor_msgs/Image</td>
             <td>16-bit gray scale encoding of the sensor Amplitude image
             normalized wrt exposure time. </td>
         </tr>
         <tr>
             <td>/o3d3xx/camera/raw_amplitude</td>
             <td>sensor_msgs/Image</td>
             <td>16-bit gray scale encoding of the sensor Amplitude image
             non-normalized.</td>
         </tr>
         <tr>
             <td>/o3d3xx/camera/cloud</td>
             <td>sensor_msgs/PointCloud2</td>
             <td>
             A 3D PCL point cloud of point type `XYZI`. In this encoding the
             intensity channel is represented by the corresponding pixel's
             amplitude data. The units of this point cloud are in meters.
             </td>
         </tr>
         <tr>
             <td>/o3d3xx/camera/confidence</td>
             <td>sensor_msgs/Image</td>
             <td>
             An 8-bit mono image encoding of the confidence image. The meaning
             of each bit of each pixel value is discussed in the official IFM
             documentation for the camera.
             </td>
         </tr>
         <tr>
             <td>/o3d3xx/camera/depth</td>
             <td>sensor_msgs/Image</td>
             <td>
             A 16-bit mono image encoding of the radial depth map from the
             camera. The depth units are in millimeters.
             </td>
         </tr>
         <tr>
             <td>/o3d3xx/camera/depth_viz</td>
             <td>sensor_msgs/Image</td>
             <td>
             A rendering of the depth image utilizing a colormap more
             human-friendly for visualization purposes. For performance
             reasons, messages are only published to this topic when the
             `publish_viz_images` parameter is set to true at launch time.
             </td>
         </tr>
         <tr>
             <td>/o3d3xx/camera/good_bad_pixels</td>
             <td>sensor_msgs/Image</td>
             <td>
             A binary image showing good vs. bad pixels on the pixel array. Bad
             pixels can be caused by numerous reasons (e.g., motion blur over
             an integration/exposure timestep). Visualizing this data is useful
             for when you are tuning your imager parameters. For performance
             reasons, messages are only published to this topic when the
             `publish_viz_images` parameter is set to true at launch time.
             </td>
         </tr>
         <tr>
             <td>/o3d3xx/camera/xyz_image</td>
             <td>sensor_msgs/Image</td>
             <td>
             An OpenCV image encoding (CV_16SC3) of the same point cloud that
             is published to `/o3d3xx/camera/cloud` where the three image planes
             are 0 = x, 1 = y, 2 = z. Units are in millimeters yet the coord
             frame is consistent with the point cloud.
             </td>
         </tr>
         <tr>
             <td>/o3d3xx/camera/unit_vectors</td>
             <td>sensor_msgs/Image</td>
             <td>
             An OpenCV image encoding (CV_32FC3) of the rotated unit vectors
             that can be used together with the translation vector from the
             camera extrinsics and the radial distance image to compute the
             cartesian coordinates for each pixel in the imager array off-board
             the camera. This topic is latched.
             </td>
         </tr>
         <tr>
             <td>/o3d3xx/camera/extrinsics</td>
             <td><a href="msg/Extrinsics.msg">Extrinsics.msg</a></td>
             <td>
             Extrinsics as reported by the camera. The translation vector here
             is used together with the unit vectors and the radial distance
             image to compute the Cartesian coordinates for each pixel in the
             imager array. It should be noted that for this message, the
             translational units are in mm and the rotational units are in
             degrees. This is to be consistent with the camera eventhough it is
             not necessarily consistent with ROS conventions. Usage of this
             message is really for very specialized use-cases.
             </td>
         </tr>
</table>

#### Advertised Services

<table>
    <tr>
        <th>Service Name</th>
        <th>Service Definition</th>
        <th>Description</th>
    </tr>
    <tr>
        <td>/o3d3xx/camera/Config</td>
        <td><a href="srv/Config.srv">Config.srv</a></td>
        <td>
        Mutates camera settings based upon an input JSON file. <b>NOTE:</b> Due
        to what appears to be limitations in the YAML parsing of the stock ROS
        `rosservice` command line tool (i.e., it does not handle JSON as string
        payload well) you will have to use the
        <i>/o3d3xx/camera/config_node</i> to configure the camera. This is
        explained in further detail below.
        </td>
    </tr>
    <tr>
        <td>/o3d3xx/camera/Dump</td>
        <td><a href="srv/Dump.srv">Dump.srv</a></td>
        <td>
        Dumps the current configuration of the camera to a JSON string. The
        output of this dump is suitable for editing and passing to the `Config`
        service for configuring the camera.
        </td>
    </tr>
    <tr>
        <td>/o3d3xx/camera/GetVersion</td>
        <td><a href="srv/GetVersion.srv">GetVersion.srv</a></td>
        <td>
        Returns the current version of the underlying
        <a href="https://github.com/lovepark/libo3d3xx">libo3d3xx</a> library
        that this ROS node is linked to.
        </td>
    </tr>
    <tr>
        <td>/o3d3xx/camera/Rm</td>
        <td><a href="srv/Rm.srv">Rm.srv</a></td>
        <td>Removes an application from the camera. This service will restrict
        removing the current active application.
        </td>
    </tr>
    <tr>
        <td>/o3d3xx/camera/Trigger</td>
        <td><a href="srv/Trigger.srv">Trigger.srv</a></td>
        <td>
        Sends a software trigger signal to the camera.
        </td>
    </tr>
</table>

#### Parameters

<table>
    <tr><th>Name</th><th>Data Type</th><th>Description</th></tr>
    <tr>
        <td>ip</td>
        <td>string</td>
        <td>IP address of the camera</td>
    </tr>
    <tr>
        <td>xmlrpc_port</td>
        <td>int</td>
        <td>TCP port the camera's XMLRPC server is listening on</td>
    </tr>
    <tr>
        <td>password</td>
        <td>string</td>
        <td>Password to use to connect to the camera</td>
    </tr>
    <tr>
        <td>schema_mask</td>
        <td>uint16_t</td>
        <td>Mask controlling which image types to stream back from the
           camera. This is useful for numerous reasons. It allows for
           conserving bandwidth between the host computer and the camera or
           lessens the CPU cycles required by the camera to compute image data
           which may result in an increased frame rate. See the o3d3xx-schema
           command line tool for generating custom masks.</td>
    </tr>
    <tr>
        <td>timeout_millis</td>
        <td>int</td>
        <td>Time, in milliseconds, to block when waiting for a frame from the
        camera before timing out.</td>
    </tr>
    <tr>
        <td>timeout_tolerance_secs</td>
        <td>double</td>
        <td>Time, in seconds, to wait before trying to restart the underlying
        framegrabber if it is currently experiencing timeouts while trying to
        capture image data.</td>
    </tr>
    <tr>
        <td>publish_viz_images</td>
        <td>bool</td>
        <td>
        In general, for a runtime system, the core data a system will want from
        this camera include the `cloud`, `depth`, `amplitude`, and `confidence`
        images. This node will always publish those data. However, if you set
        this parameter to `true` a few additional images are published. These
        are `depth_viz` and `good_bad_pixels` (they are described
        above in the `Topics` section). These <i>viz images</i> are intended
        for human analysis and visualization in `rviz`.
        </td>
    </tr>
    <tr>
        <td>assume_sw_trigger</td>
        <td>bool</td>
        <td>
        If your application intends to use software triggering (as opposed to a
        free-running framegrabber), this parameter does four things. First, it
        will not print out framegrabber timeout messages to the
        rosconsole. Second, it will set, a-priori, the `timeout_millis`
        parameter to 500 milliseconds (this is so that the node will release
        the framegrabber mutex in the main publishing loop). Third, this will
        set the `timeout_tolerance_secs` to 600 seconds so that the
        framegrabber is not constantly getting recycled unnecessarily. Fourth,
        it sets the `GLOG_minloglevel` environment variable to 3 (ERROR). This
        is so the underlying `libo3d3xx` library does not fill your log files
        with timeout messages which it emits at the WARNING level.
        </td>
    </tr>
    <tr>
        <td>frame_id_base</td>
        <td>string</td>
        <td>tf frame prefix</td>
    </tr>
</table>


### /o3d3xx/camera_tf

This node is of type `tf/static_transform_publisher`. It establishes a frame_id
for the camera in the global tf tree. This node is launched from the primary
`camera.launch` file:

    $ roslaunch o3d3xx camera.launch

When run as above, the tf publishing node would be named `/o3d3xx/camera_tf`
and the camera coordinate frame would be `o3d3xx/camera_link` in the tf tree.

You can customize this naming via the `frame_id_base` arg or implicitly through
the `ns` (namespace) and `nn` (node name) command line arguments passed to the
`camera.launch` file. For example, if you specify your roslaunch command as:

    $ roslaunch o3d3xx camera.launch ns:=robot nn:=front_camera

The node name will be `/robot/front_camera_tf` and the camera frame will
be `robot/front_camera_link` in the tf tree.

### /o3d3xx/camera/config_node

This node is used as a proxy to simplify calling the `/o3d3xx/camera/Config`
service offered by the `/o3d3xx/camera` node. It was noted above that there
appears to be a limitation in the YAML parser of the ROS `rosservice` command
line tool. Specifically, it seems that it is not capable of assigning a JSON
string to a variable. This is the reason for this node. This is not a
long-running node but rather works like a typical command-line tool would: you
invoke it, it runs, and exits. The following command line will launch this
node:

    $ roslaunch o3d3xx config.launch

#### Parameters

<table>
    <tr><th>Name</th><th>Data Type</th><th>Description</th></tr>
    <tr>
        <td>infile</td>
        <td>string</td>
        <td>
        By default, this node will read `stdin` for a JSON string to use to
        pass to the `/o3d3xx/camera/Config` service. However, if this parameter
        is specified it will read the JSON from this file.
        </td>
    </tr>
</table>

### /rviz

This package offers a launch script that wraps the execution of `rviz` so that
the display will be conveniently configured for visualizing the
`/o3d3xx/camera` data. To launch this node:

    $ roslaunch o3d3xx rviz.launch

Running the command as above will color the point cloud with the
data from the normalized amplitude image (i.e., the intensity).

The rviz window should look something like (assuming you are coloring the point
cloud with the intensity data):

![rviz1](doc/figures/rviz1.png)

### /o3d3xx/camera/XXX_throttler

This package offers a launch script that wraps the
[topic_tools/throttler](http://wiki.ros.org/topic_tools/throttle) node
so that it can throttle the core topics from the camera. Specifically, it will
throttle `/o3d3xx/camera/cloud` to `/o3d3xx/camera/cloud_throttle`,
`/o3d3xx/camera/amplitude` to `/o3d3xx/camera/amplitude_throttle`,
`/o3d3xx/camera/raw_amplitude_throttle`,
`/o3d3xx/camera/depth` to `/o3d3xx/camera/depth_throttle`,
`/o3d3xx/camera/confidence` to `/o3d3xx/camera/confidence_throttle`. To launch
this node:

    $ roslaunch o3d3xx throttled.launch

By default, it will throttle the above named topics to 1 Hz. You can change the
frequency with the `hz` command line argument. For example, to send data at 2
Hz:

    $ roslaunch o3d3xx throttled.launch hz:=2.0

Using this launch file to launch this set of nodes is strictly optional. We
have found use for it in two ways. First, to slow down the publishing frequency
of the topics when used in conjunction with the `/o3d3xx/camera/file_writer`
node for collecting data (i.e., in those instances when we really do not need
all the data but rather some subsampling of it). Second, if we are running the
camera on a drone (for example) that has a slower radio link down to a ground
control station running `rviz` where we want to see what the camera sees while
the drone is in flight. Clearly there are other uses for this, YMMV.


Configuring Camera Settings
---------------------------

Configuring the camera is accomplished by passing a JSON string to the
`/o3d3xx/camera/config_node` which will call the `/o3d3xx/camera/Config`
service to mutate the camera settings. Using a JSON string to configure the
camera has the following primary benefits:

0. Configuration is declarative. The camera configuration will reflect that
   which is described by the JSON file.
1. The JSON data is human-consumable and easily edited in a text editor. This
   makes it very convenient for headless embedded systems.
2. The JSON data is machine parsable, so configuring the camera on the fly via
   programmable logic is also possible.

There are also a few downfalls to using JSON. Most notably the lack of comments
and an enforceable schema. One could argue that the latter keeps things
flexible. None-the-less, JSON is the format used by `libo3d3xx` and, by
extension, this ROS package.

An exemplary JSON file is shown [here](json/ex_camera.json) (this is the result
of calling the `/o3d3xx/camera/Dump` service on a development system). When
passing a JSON string (like the previously linked to file) to the
`/o3d3xx/camera/Config` service (or to the `/o3d3xx/camera/config_node`) the
following rules are used to configure the camera:

0. The `Device` section is processed and saved on the camera.
1. The `Apps` section is processed. For each app:
  0. If the `Index` key is present, a current app at that `Index` is looked
     up. If present, it is edited to reflect the data in the JSON file. If an
     app at that `Index` is not present, a new app is created with the
     parameters from the JSON file. It is not guaranteed that the new app will
     have the specified `Index`.
  1. If the `Index` key is not present, a new app is created with the
     parameters as specified in the JSON file.
2. The active application is set by consulting the desired index of the
   `ActiveApplication` from the `Device` section of the JSON. If the specified
   `Index` does not exist, the active application is not set.
3. The `Net` section is processed. A reboot of the camera may be necessary
   after changing the camera's network parameters. Additionally, you will
   likely need to restart the `/o3d3xx/camera` node pointing it to the new IP
   address (if that is what you changed).

It should also be noted that any portion of the JSON tree can be specfied to
configure only that part of the camera. The only rule to follow is that all
keys should be fully qualified. For example, to simply set the active
application, you can use a JSON snippet like this:

    {
        "o3d3xx":
        {
            "Device":
            {
                "ActiveApplication": "2"
            }
        }
    }

The above snippet is provided as an example [here](json/ex_set_active.json). To
apply this to your camera, you can:

    $ roslaunch o3d3xx config.launch infile:=/path/to/ex_set_active.json

It was also noted above that the `/o3d3xx/camera/config_node` will read `stdin`
by default, so you could also:

    $ echo '{"o3d3xx":{"Device":{"ActiveApplication":"2"}}}' | roslaunch o3d3xx config.launch

[Here](json/ex_add_app.json) is another example JSON file. This one will add a
new application to the camera, using the default values for the high-dynamic
range imager. We note that this application is _added_ to the camera because no
`Index` is specified for the application. If an `Index` were specfied, the
application at the specified `Index`, if present, would be edited to reflect
this configuration.

In general, a simple way to configure camera settings without having to
memorize the JSON syntax would be to simply dump the current camera settings to
a file:

    $ rosservice call /o3d3xx/camera/Dump > /tmp/camera.json

Then, open `/tmp/camera.json` with a text editor to create a declarative JSON
configuration for your camera. You should be sure to delete the variable names
from the `rosservice` output if you are following this example
_word-for-word_. Additionally, you can delete any unnecessary keys if you would
like, however it is not strictly necessary as the `/o3d3xx/camera/Config`
service will leave unedited values unchanged on the camera. Once you have a
configuration that you like, you can:

    $ roslaunch o3d3xx config.launch infile:=/tmp/camera.json

You can check that your configuration is active by calling the
`/o3d3xx/camera/Dump` service again.

TODO
----

Please see the [Github Issues](https://github.com/lovepark/o3d3xx-ros/issues).

LICENSE
-------

Please see the file called LICENSE.

AUTHORS
-------

Tom Panzarella <tom@loveparkrobotics.com>
