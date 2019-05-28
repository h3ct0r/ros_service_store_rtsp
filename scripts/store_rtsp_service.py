#! /usr/bin/env python

import rospy
from ros_service_store_rtsp.srv import StoreRTSP, StoreRTSPResponse
from urlparse import urlparse
import glob
import datetime
import re
import os
import time
from stream_proc import StreamProc
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class StoreStreamService:
    """
    Service to manage the start/stop of the streaming recorders
    """

    def __init__(self, base_output_path='/tmp', extension='.mkv', video_bitrate='900k', video_format="matroska",
                 fps=15):
        self.base_output_path = base_output_path
        self.extension = extension
        self.video_bitrate = video_bitrate
        self.fps = fps
        self.video_format = video_format
        self.process_dict = {}

    def get_output_filename(self, stream_uri):
        """
        Define new output filename for the streamed video
        Given a folder and a extension name generate the next sequence with a timestamp
        :return:
        """

        files = [os.path.basename(x) for x in glob.glob(
            os.path.join(self.base_output_path, "*{}".format(self.extension)))]

        exp_size = 0
        if len(files) > 0:
            v_list = []
            for f in files:
                try:
                    v_list.append(int(f.split('.')[0]))
                except ValueError:
                    pass
            if len(v_list) > 0:
                exp_size = max(v_list)

        current_timestamp = datetime.datetime.now().strftime("%Y%m%d.%H%M%S")

        filename = os.path.join(self.base_output_path, "{}.{}.{}{}".format(exp_size + 1, self.sanitize_uri(stream_uri),
                                                                            current_timestamp, self.extension))

        rospy.loginfo("Generated new video filepath: %s", filename)
        return filename

    def sanitize_uri(self, uri):
        """
        Get a clean uri without special characters
        :param uri:
        :return:
        """
        pattern = re.compile('[\W_]+', re.UNICODE)
        return pattern.sub('_', uri)

    def store_response_callback(self, request):
        """
        Callback function used by the service server to process
        requests from clients. It returns a StoreRTSPResponse object
        This method uses a ffmpeg wrapper to store the streams seamesly
        A shared dict object stores the processes of the multiple ffmpeg streams
        """

        rospy.loginfo("Received %s", request)

        is_stream_already_processed = (request.stream_uri in self.process_dict.keys())
        if not request.is_to_store:
            if is_stream_already_processed:
                rospy.loginfo("Stopping stream: %s", request.stream_uri)

                p_obj = self.process_dict.pop(request.stream_uri, None)
                is_stop_successful, msg = p_obj.stop_recording()

                if not is_stop_successful:
                    rospy.loginfo("Exception stopping stream: %s %s", request.stream_uri, msg)
                    return StoreRTSPResponse(
                        success=False,
                        msg=msg
                    )
                else:
                    rospy.loginfo("Stopped stream successfully: %s %s", request.stream_uri, msg)
                    return StoreRTSPResponse(
                        success=True,
                        msg="Stopped stream successfully: {}".format(request.stream_uri),
                        filename=p_obj.get_output_filepath(),
                        duration_secs=p_obj.get_video_duration(),
                        wallclock_secs=int(time.time() - p_obj.get_start_time())
                    )
            else:
                rospy.loginfo("Cannot stop stream, stream not found: %s", request.stream_uri)
                return StoreRTSPResponse(
                    success=False,
                    msg="Cannot stop stream, stream not found ({})".format(request.stream_uri)
                )

        else:
            if is_stream_already_processed:
                rospy.loginfo("Cannot start stream, stream already beign processed: %s", request.stream_uri)
                return StoreRTSPResponse(
                    success=False,
                    filename="",
                    msg="Stream already beign processed {}".format(request.stream_uri)
                )
            else:
                output_filepath = self.get_output_filename(request.stream_uri)

                # define a simple, straightforward name for the publisher
                parsed_uri = urlparse(request.stream_uri)
                if parsed_uri.hostname:
                    published_uri = parsed_uri.hostname
                    if parsed_uri.port != '':
                        published_uri += ':{}'.format(parsed_uri.port)
                else:
                    published_uri = self.sanitize_uri(os.path.basename(request.stream_uri))

                rospy.loginfo("Publishing images at topic: %s", published_uri)
                frame_publisher = rospy.Publisher(published_uri, Image, queue_size=10)

                stream_proc = StreamProc(request.stream_uri, output_filepath, frame_publisher)
                is_recording = stream_proc.start_recording()

                if is_recording:
                    rospy.loginfo("Recording stream: %s", request.stream_uri)
                    self.process_dict[request.stream_uri] = stream_proc

                    return StoreRTSPResponse(
                        success=True,
                        filename=output_filepath,
                        msg="Recording stream {}".format(request.stream_uri)
                    )
                else:
                    rospy.loginfo("Exception starting stream: %s %s", request.stream_uri, stream_proc.get_err())
                    return StoreRTSPResponse(
                        success=False,
                        filename=output_filepath,
                        msg=stream_proc.get_err()
                    )

    def extract_last_frame_from_streams(self):
        """
        Extract the last saved frame from the stored streams
        and publish them as images
        :return:
        """

        for k in self.process_dict.keys():
            p = self.process_dict[k]
            p.extract_fps_from_stream()

            last_frame = p.extract_last_frame_from_stream()
            if last_frame is not None:
                try:
                    msg_frame = CvBridge().cv2_to_imgmsg(last_frame)
                    frame_pub = p.get_frame_publisher()
                    frame_pub.publish(msg_frame)
                except CvBridgeError as e:
                    rospy.logerr("Error converting frame to ROS format: %s %s", p.get_stream_uri(), e)

    def stop_all_recordings(self):
        """
        Stop all recordings currently opened
        :return:
        """

        for k in self.process_dict.keys():
            p = self.process_dict[k]
            rospy.loginfo("Stopping stream %s", p.get_stream_uri())
            p.stop_recording()


if __name__ == "__main__":
    rospy.init_node('store_rtsp_service')

    arg_base_output_path = rospy.get_param('~base_output_path', '/tmp')
    arg_extension = rospy.get_param('~extension', '.mkv')
    arg_video_bitrate = rospy.get_param('~video_bitrate', '900k')
    arg_fps = rospy.get_param('~fps', 15)
    arg_format = rospy.get_param('~format', 'matroska')
    arg_publish_screenshots = rospy.get_param('~publish_screenshots', True)

    store_service = StoreStreamService(
        base_output_path=arg_base_output_path,
        extension=arg_extension,
        video_bitrate=arg_video_bitrate,
        fps=arg_fps,
        video_format=arg_format
    )

    rospy.Service('/store_rtsp', StoreRTSP, store_service.store_response_callback)

    r = rospy.Rate(0.05)
    while not rospy.is_shutdown():
        if arg_publish_screenshots:
            # at the specified interval extract an actual frame
            # of the current streams and publish tem as ROS images
            store_service.extract_last_frame_from_streams()

        r.sleep()

    # clean all video stream processes before exit
    rospy.loginfo("Stopping all video streams before exiting")
    store_service.stop_all_recordings()
