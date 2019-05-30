# ROS Service to store RTSP streams (ros_service_store_rtsp)
This is a Python ROS service to save RTSP stream to a file 'the easy way' without dealing with a lot of ffmpeg headaches.

### Installation

Dillinger requires Ubuntu 16.04, `ffmpeg`, `numpy`, `ffmpeg-python` and `cv_bridge` to run.

Install the dependencies (the easy way!):

```sh
$ cd ~/catkin_ws/src
$ git clone https://github.com/h3ct0r/ros_service_store_rtsp
$ cd ~/catkin
$ echo "yaml file://$(readlink -f src/ros_service_store_rtsp/deps/custom_rosdep_rules.yaml)" | sudo tee -a /etc/ros/rosdep/sources.list.d/20-default.list
$ rosdep update
$ rosdep install --from-paths src/ros_service_store_rtsp --ignore-src -r -y 
```

### How to run

#### Launch

Launch the main launch file:

```sh
$ roslaunch ros_service_store_rtsp store_rtsp_service.launch
```

And the service will show up:

`rosservice list` will show the `/store_rtsp` service.

#### Input
The service is called with a StoreRTSPRequest message type, that is basically a boolean ("start or stop" recording) and a string (The camera URI such as the RSTP address or a file such as /dev/video0). 

```python
#!/usr/bin/env python
from ros_service_store_rtsp.srv import StoreRTSPRequest, StoreRTSPResponse
store_request = StoreRTSPRequest(True, "rtsp://stream.com/live/live.sdp")
```

```sh
$ rosservice call /store_rtsp true "/dev/video0" # to start recording
$ rosservice call /store_rtsp false "/dev/video0" # to stop recording
```

#### Output

The output is a StoreRTSPResponse object with the following data:

- bool success : if the request/response was a success or not
- string filename : the filepath of the video file being processed
- string msg : a status message of the request
- uint16 duration_secs : the duration of the video calculated by ffmpeg
- uint16 wallclock_secs : the wallclock seconds between the video recording and stopping the recording

#### Configuring the launch file:

A tipical launch file look like this:

```xml
<launch>
    <arg name="group_name" default="store_video" />
    <group ns="$(arg group_name)">
        <node pkg="ros_service_store_rtsp" type="store_rtsp_service.py" name="store_rtsp_service" output="screen">
            <param name="base_output_path" value="$(find ros_service_store_rtsp)/stored_streams" />
            <param name="extension" value=".mkv" />
            <param name="format" value="matroska" />
            <param name="fps" value="15" />
            <param name="video_bitrate" value="900k" />
            <param name="publish_screenshots" value="true"/>
        </node>
    </group>
</launch>
```

Where: 

- base_output_path: is the base directory to store the videos (`/tmp` is the default)
- extension: the extension of the filename (`mkv` is the default)
- format: video format (`matroska` is the default)
- fps: frame per second stored at the output video (15 fps is the default)
- video_bitrate: the quality of the video beign stored, `900k` is the default.
- publish screenshots: a boolean to publish a screenshot of the video beign processed (usefull to check if the video is beign stored withoput problems). The default is `True`.

### Development

Want to contribute? Great! Send me a pull request with your changes!

#### Testing

Execute this command to run the tests:

```sh
$ rostest ros_service_store_rtsp test_stream.test
```

### Todos

 - Write MORE Tests
 - test with more cameras

License
----

MIT


**Free Software, Hell Yeah!**
