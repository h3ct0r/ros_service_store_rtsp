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
import re
from threading import Thread


class StoreStreamService:
    """
    Service to manage the start/stop of the streaming recorders
    """

    def __init__(self, base_output_path='/tmp', extension='.mp4', video_bitrate='900k', acodec="copy",
                 vcodec="copy", fps=15, segment_time=300, segment_format="matroska"):
        self.base_output_path = base_output_path
        self.extension = extension
        self.video_bitrate = video_bitrate
        self.fps = fps
        self.acodec = acodec
        self.vcodec = vcodec
        self.process_dict = {}
        self.segment_time = segment_time
        self.segment_format = segment_format

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

        current_timestamp = datetime.datetime.now().strftime("%Y-%m-%d.%H.%M.%S")

        exp_size += 1
        filename = os.path.join(self.base_output_path, "{}.{}.{}{}".format(exp_size, self.simplify_uri(stream_uri),
                                                                            current_timestamp, self.extension))

        rospy.loginfo("Generated new video filepath: %s", filename)
        return filename, exp_size

    def sanitize_uri(self, uri):
        """
        Get a clean uri without special characters
        :param uri:
        :return:
        """
        pattern = re.compile('[\W_:]+', re.UNICODE)
        return pattern.sub('_', uri)

    def simplify_uri(self, uri, prefix=None):
        """
        Simplify a complex uri with a small and nice URI to publish and save files
        :param uri:
        :return:
        """
        parsed_uri = urlparse(uri)
        if parsed_uri.hostname:
            published_uri = parsed_uri.hostname
            if parsed_uri.port != '':
                published_uri += ':{}'.format(parsed_uri.port)

            published_uri = self.sanitize_uri(published_uri)
        else:
            published_uri = self.sanitize_uri(os.path.basename(uri))

        stream_pre = "stream_"
        if prefix is not None:
            stream_pre += str(prefix) + "_"

        return stream_pre + published_uri

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
                output_filepath, video_idx = self.get_output_filename(request.stream_uri)
                published_uri = self.simplify_uri(request.stream_uri, prefix=video_idx)

                rospy.loginfo("Saving video to: %s", output_filepath)
                rospy.loginfo("Publishing images at topic: %s", published_uri)

                frame_publisher = rospy.Publisher(published_uri, Image, queue_size=10)

                stream_proc = StreamProc(request.stream_uri, output_filepath, frame_publisher, extension=self.extension,
                                         acodec=self.acodec, vcodec=self.vcodec, fps=self.fps,
                                         video_bitrate=self.video_bitrate, segment_time=self.segment_time,
                                         segment_format=self.segment_format)
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

    def extract_frame_from_streams(self):
        """
        Extract the last saved frame from the stored streams
        and publish them as images
        :return:
        """
        publish_threads = []

        for k in self.process_dict.keys():
            p = self.process_dict[k]
            local_t = Thread(target=self.extract_and_publish_frame_proc, args=(p, ))
            local_t.start()
            publish_threads.append(local_t)

        for t in publish_threads:
            t.join()

    def extract_and_publish_frame_proc(self, p):
        """
        Separate the function that extract the images from the stream
        to use it as a thread
        :param p:
        :return:
        """
        last_frame = p.extract_frame_from_stream()
        if last_frame is not None:
            try:
                msg_frame = CvBridge().cv2_to_imgmsg(last_frame)
                frame_pub = p.get_frame_publisher()
                frame_pub.publish(msg_frame)
            except CvBridgeError as e:
                rospy.logerr("Error converting frame to ROS format: %s %s", p.get_stream_uri(), e)
        else:
            rospy.logwarn("Last frame of stream %s is None (frame number: %s)", p.get_stream_uri(),
                          p.get_last_frame_number())

    def stop_all_recordings(self):
        """
        Stop all recordings currently opened
        :return:
        """
        stop_threads = []

        for k in self.process_dict.keys():
            p = self.process_dict[k]

            local_t = Thread(target=self.stop_recording_proc, args=(p, ))
            local_t.start()
            stop_threads.append(local_t)

        for t in stop_threads:
            t.join()

    def stop_recording_proc(self, p):
        """
        Separate the function that stop the processes
        to use it as a thread
        :param p:
        :return:
        """
        rospy.loginfo("Stopping stream %s", p.get_stream_uri())
        p.stop_recording()

    def clean_all_pipes(self):
        """
        Clean all pipes from the opened processes
        :return:
        """
        pipe_threads = []

        for k in self.process_dict.keys():
            p = self.process_dict[k]

            local_t = Thread(target=self.clean_pipe_proc, args=(p, ))
            local_t.start()
            pipe_threads.append(local_t)

        for t in pipe_threads:
            t.join()

    def clean_pipe_proc(self, p):
        """
        Separate the function to clean the pipes of the process
        to use it as a thread
        :param p:
        :return:
        """
        p.extract_fps_from_ffmpeg_output()
        p.get_stdout_output()
        p.get_stderr_output()


if __name__ == "__main__":
    rospy.init_node('store_rtsp_service')

    arg_base_output_path = rospy.get_param('~base_output_path', '/tmp')
    arg_extension = rospy.get_param('~extension', '.mkv')
    arg_video_bitrate = rospy.get_param('~video_bitrate', '900k')
    arg_fps = rospy.get_param('~fps', 15)
    arg_acodec = rospy.get_param('~acodec', '')
    arg_vcodec = rospy.get_param('~vcodec', 'copy')
    arg_publish_screenshots = rospy.get_param('~publish_screenshots', True)
    publish_screenshots_rate_seconds = rospy.get_param('~publish_screenshots_rate_seconds', 1)
    arg_segment_time = rospy.get_param('~segment_time', 300)
    arg_segment_format = rospy.get_param('~segment_format', "matroska")

    if publish_screenshots_rate_seconds < 1:
        rospy.logwarn("Publish screenshot rate less than 1: {}, setting it to 1".format(publish_screenshots_rate_seconds))
        publish_screenshots_rate_seconds = 1

    if not os.path.exists(arg_base_output_path):
        rospy.logwarn("Creating directory to store strams: {}".format(arg_base_output_path))
        try:
            os.makedirs(arg_base_output_path)
        except OSError:
            if not os.path.isdir(arg_base_output_path):
                raise

    store_service = StoreStreamService(
        base_output_path=arg_base_output_path,
        extension=arg_extension,
        video_bitrate=arg_video_bitrate,
        fps=arg_fps,
        acodec=arg_acodec,
        vcodec=arg_vcodec,
        segment_time=arg_segment_time,
        segment_format=arg_segment_format
    )

    rospy.Service('/store_rtsp', StoreRTSP, store_service.store_response_callback)

    if arg_publish_screenshots:
        rospy.loginfo("Publishing image topics:{} at second interval:{}".format(arg_publish_screenshots,
                                                                                publish_screenshots_rate_seconds))
    else:
        rospy.logwarn("Not publishing image topics param 'publish_screenshots_rate_seconds' set to false")

    r = rospy.Rate(2)
    frame_extract_timeout = time.time()

    while not rospy.is_shutdown():
        # clean the pipes to avoid freezing the subprocess when
        # running out of memory from the pipes without reading them (4k)
        # https://stackoverflow.com/questions/16523746/ffmpeg-hangs-when-run-in-background
        store_service.clean_all_pipes()

        if arg_publish_screenshots and time.time() - frame_extract_timeout < publish_screenshots_rate_seconds:
            # at the specified interval extract an actual frame
            # of the current streams and publish them as ROS images
            store_service.extract_frame_from_streams()
            frame_extract_timeout = time.time()

        r.sleep()

    # clean all video stream processes before exit
    rospy.loginfo("Stopping all video streams before exiting")
    store_service.stop_all_recordings()
