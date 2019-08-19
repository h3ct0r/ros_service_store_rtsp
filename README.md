# ROS Service to store RTSP streams (ros_service_store_rtsp)
This is a Python ROS service to save RTSP stream to a file 'the easy way' without dealing with a lot of ffmpeg headaches.

### Installation

Dillinger requires Ubuntu 16.04, `ffmpeg`, `numpy`, `ffmpeg-python` and `cv_bridge` to run.

Install the dependencies (the easy way!):

```bash
$ cd ~/catkin_ws/src
$ git clone https://github.com/h3ct0r/ros_service_store_rtsp
$ cd ~/catkin_ws
$ echo "yaml file://$(readlink -f src/ros_service_store_rtsp/deps/custom_rosdep_rules.yaml)" | sudo tee -a /etc/ros/rosdep/sources.list.d/20-default.list
$ rosdep update
$ sudo rosdep install --from-paths src/ros_service_store_rtsp --ignore-src -r -y 
```
Finally install the latest version of ffmpeg (4.1 for Ubuntu 16.04 using the PPA). This is optinal, but **recommended**.
```bash
$ sudo add-apt-repository ppa:jonathonf/ffmpeg-4 -y
$ sudo apt update
$ sudo apt install ffmpeg -y
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
from ros_service_store_rtsp.srv import StoreRTSP

def callStoreService(status, link):
    rospy.wait_for_service('/store_rtsp')
    try:
        store_request = rospy.ServiceProxy('/store_rtsp', StoreRTSP)
        print store_request(status, link)

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

callStoreService(True, "rtsp://stream.com/live/live.sdp")
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
            <param name="acodec" value="" /> <!-- leave empty or do not set this param to disable audio -->
            <param name="vcodec" value="copy" />
            <param name="fps" value="15" />
            <param name="video_bitrate" value="900k" />
            <param name="publish_screenshots" value="true"/>
            <param name="publish_screenshots_rate" value="1"/>
            <param name="segment_time" value="300"/>
            <param name="segment_format" value="matroska"/>
        </node>
    </group>
</launch>
```

Where: 

- base_output_path: is the base directory to store the videos (`/tmp` is the default)
- extension: the extension of the filename (`mkv` is the default)
- acodec: audio codec
- vcodec: video codec
- fps: frame per second stored at the output video (15 fps is the default)
- video_bitrate: the quality of the video beign stored, `900k` is the default.
- publish screenshots: a boolean to publish a screenshot of the video beign processed (usefull to check if the video is beign stored withoput problems). The default is `True`.
- publish_screenshots_rate: rate at which the images are published
- segment_time: size of a video segment
- segment format: video format of the segment (default to `matroska`)

### Merge multiple video files

The resulting segments can be stitched by first creating a text file `seg.txt` like this

```
file 'encoded_testfile_piece_00.mp4'
file 'encoded_testfile_piece_01.mp4'
file 'encoded_testfile_piece_02.mp4'
file 'encoded_testfile_piece_03.mp4'
```

And then running

`ffmpeg -f concat -i seg.txt -c copy -fflags +genpts encoded_full.mp4`

More info at [https://stackoverflow.com/questions/7333232/how-to-concatenate-two-mp4-files-using-ffmpeg](https://stackoverflow.com/questions/7333232/how-to-concatenate-two-mp4-files-using-ffmpeg)

### Development

Want to contribute? Great! Send me a pull request with your changes!

#### Testing

Execute this command to run the tests:

```sh
$ rostest ros_service_store_rtsp test_stream.test
```

Example ffmpeg command executed:

```bash
ffmpeg -rtsp_transport tcp -stimeout 3000000 -i rtsp://user:pass@192.168.1.2:554/stream0 -f segment -b:v 900k -an -flags +global_header -map 0 -map_metadata -1 -movflags +frag_keyframe+separate_moof+omit_tfhd_offset+empty_moov -reset_timestamps 1 -segment_format matroska -segment_time 300 -strict 2 -vcodec copy /tmp/1.stream.mkv -y
```

### Todos

 - Write MORE Tests
     - Test function to calculate video duration for all segments
     - Service entry to get the number of segments and duration so far
 - Check if the ffmpeg subprocess has died and restart it for every stream
 - test with more cameras

License
----

MIT


**Free Software, Hell Yeah!**
