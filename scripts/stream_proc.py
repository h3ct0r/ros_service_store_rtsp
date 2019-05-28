#! /usr/bin/env python

import rospy
import time
from urlparse import urlparse
import ffmpeg
import datetime
import signal
import shutil
import tempfile
import os
import fcntl
import re
import numpy as np
import cv2


class StreamProc:
    """
    Object to manage the storage of streams
    Uses FFmpeg wrapper to store and process video streams
    This setup has been tested with USB video and RTSP streams
    """

    def __init__(self, stream_uri, output_filepath, frame_publisher, extension=".mkv", video_format="matroska", fps="15",
                 video_bitrate="900k", stimeout=3000000):
        self.stream_uri = stream_uri
        self.output_filepath = output_filepath
        self.frame_publisher = frame_publisher
        self.start_time = 0
        self.duration = 0
        self.metadata = None
        self.proc = None
        self.out = None
        self.err = None
        self.last_frame_number = 0

        self._def_timeout_secs = 4.0
        self._def_extension = extension
        self._def_video_format = video_format
        self._def_fps = fps
        self._def_bitrate = video_bitrate
        self._def_stimeout = stimeout  # stimeout in microseconds
        self._def_min_video_size_bytes = 5000  # check if the output is at least some bytes before return success

    def start_recording(self):
        """
        Start recording the stream defined on self.stream_uri
        :return: boolean if the recording started sucessfully
        """

        # if we are dealing with a USB camera or RTSP stream we
        # treat them differently, for example remove the socket TCP I/O timeout (stimeout)
        parsed_uri = urlparse(self.stream_uri)
        if parsed_uri.scheme == "rtsp":
            stream = ffmpeg.input(self.stream_uri, stimeout=self._def_stimeout)  # stimeout in microseconds
            stream = ffmpeg.filter(stream, 'fps', fps=self._def_fps, round='up')
            stream = ffmpeg.output(stream, self.output_filepath, format=self._def_video_format,
                                   video_bitrate=self._def_bitrate)
            stream = ffmpeg.overwrite_output(stream)
        else:
            stream = ffmpeg.input(self.stream_uri)
            stream = ffmpeg.output(stream, self.output_filepath, format=self._def_video_format,
                                   video_bitrate=self._def_bitrate)
            stream = ffmpeg.overwrite_output(stream)

        test = ffmpeg.compile(stream)
        print(test)
        print(' '.join(test))

        self.start_time = time.time()
        self.proc = (
            ffmpeg.run_async(stream, pipe_stdout=True, pipe_stderr=True)
        )

        start_timeout = time.time()
        is_reached_size = False
        while self.proc.poll() is None and time.time() - start_timeout < self._def_timeout_secs and not is_reached_size:
            try:
                if os.path.getsize(self.output_filepath) > self._def_min_video_size_bytes:
                    is_reached_size = True
            except OSError as e:
                pass
            time.sleep(0.1)

        if self.proc.poll() is None:
            return True
        else:
            self.start_time = 0
            self.out, self.err = self.proc.communicate()
            return False

    def stop_recording(self):
        """
        Stops the stream by first sending a ctrl+c signal and then killing the process after a timeout
        :return: boolean, string. First parameter returns the status of the stop and the status message
        """

        # send a ctrl+c signal to gracefully stop the process
        try:
            self.proc.send_signal(signal.SIGINT)
        except OSError as e:
            print "Error stopping ffmpeg for uri:{} {}".format(self.stream_uri, e)

        # check if the process is finished
        start_timeout = time.time()
        while self.proc.poll() is None and time.time() - start_timeout < self._def_timeout_secs:
            time.sleep(0.1)

        # force terminate the process
        if self.proc.poll() is None:
            self.proc.terminate()

        # fix random problems with metadata on the video
        # using a simmilar approach to: https://video.stackexchange.com/questions/18220/fix-bad-files-and-streams-
        # with-ffmpeg-so-vlc-and-other-players-would-not-crash
        # original command: ffmpeg -err_detect ignore_err -i video.mkv -c copy video_fixed.mkv
        _, temp_file_path = tempfile.mkstemp(suffix='.{}'.format(self._def_extension))
        stream = ffmpeg.input(self.output_filepath, err_detect="ignore_err")
        stream = ffmpeg.output(stream, temp_file_path, c="copy")
        stream = ffmpeg.overwrite_output(stream)
        ffmpeg.run(stream, pipe_stdout=True, pipe_stderr=True)

        # after fixing the video on a random tempfile, delete the original video and
        # move the new one to the previous location
        try:
            os.remove(self.output_filepath)
        except OSError:
            pass
        else:
            shutil.move(temp_file_path, self.output_filepath)

        if not os.path.isfile(self.output_filepath):
            return False, "Stored video not found at desired location: {}".format(self.output_filepath)

        self.update_video_duration()
        return True, ""

    def extract_metadata(self):
        """
        Uses ffmpeg.probe to extract the metadata of the video at the self.output_filepath file
        :return:
        """

        # extract video metadata
        try:
            probe = ffmpeg.probe(self.output_filepath)
            self.metadata = next((stream for stream in probe['streams'] if stream['codec_type'] == 'video'), None)
        except:
            pass

    def update_video_duration(self):
        """
        Uses the metadata of the video to calculate the duration
        :return:
        """

        self.extract_metadata()

        # extract video duration and parse it to seconds
        # ffmpeg duration format is "00:04:04.199000000"
        self.duration = 0
        if 'tags' in self.metadata.keys() and 'DURATION' in self.metadata['tags'].keys():
            str_duration = self.metadata['tags']['DURATION']
            x = time.strptime(str_duration.split('.')[0], '%H:%M:%S')
            self.duration = int(datetime.timedelta(hours=x.tm_hour, minutes=x.tm_min, seconds=x.tm_sec) \
                                .total_seconds())

    def _non_block_read(self, output):
        """
        Read the buffer of the stdout stream without blocking
        :param output:
        :return:
        """

        fd = output.fileno()
        fl = fcntl.fcntl(fd, fcntl.F_GETFL)
        fcntl.fcntl(fd, fcntl.F_SETFL, fl | os.O_NONBLOCK)
        try:
            return output.read()
        except:
            return ""

    def extract_fps_from_stream(self):
        """
        Read the output of ffmpeg form the stdout in a nonblocking way
        and then parse this ouput to extract the current frame
        :return:
        """

        output = self._non_block_read(self.proc.stderr)

        d = {}
        for line in output.splitlines():
            if 'frame' in line:
                regex_res = re.findall(r'[a-z]+\s*=\s*[\d\.a-zA-Z\/\:]+', line)
                for elem in regex_res:
                    f_elem = [e.strip() for e in elem.split("=")]
                    if len(f_elem) == 2:
                        d[f_elem[0]] = f_elem[1]

        if "frame" in d.keys():
            self.last_frame_number = int(d['frame'])

    def extract_last_frame_from_stream(self, frame_offset=60):
        """
        Extract the last frame of the stream beign recorded directly from the output file
        :return: np frame
        """

        if self.metadata is None:
            self.extract_metadata()
            return None

        width = int(self.metadata['width'])
        height = int(self.metadata['height'])

        last_frame_idx = self.last_frame_number - frame_offset
        if last_frame_idx < 0:
            last_frame_idx = 0

        out, _ = (
            ffmpeg
                .input(self.output_filepath)
                .filter('select', 'gte(n,{})'.format(last_frame_idx))
                .output('pipe:', vframes=1, format='rawvideo', pix_fmt='rgb24')
                .run(capture_stdout=True, capture_stderr=True)
        )

        video_stream = (
            np.frombuffer(out, np.uint8).reshape([-1, height, width, 3])
        )

        if video_stream.shape[0] == 1:
            RGB_img = cv2.cvtColor(video_stream[0], cv2.COLOR_BGR2RGB)
            return RGB_img
        else:
            return None

    def get_video_duration(self):
        return self.duration

    def get_err(self):
        return self.err

    def get_output_filepath(self):
        return self.output_filepath

    def get_start_time(self):
        return self.start_time

    def get_last_frame_number(self):
        return self.last_frame_number

    def get_frame_publisher(self):
        return self.frame_publisher

    def get_stream_uri(self):
        return self.stream_uri


if __name__ == "__main__":
    pass