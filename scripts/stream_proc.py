#! /usr/bin/env python

import datetime
import fcntl
import glob
import os
import re
import signal
import time
from collections import OrderedDict
from urlparse import urlparse

import cv2
import ffmpeg
import numpy as np


class StreamProc:
    """
    Object to manage the storage of streams
    Uses FFmpeg wrapper to store and process video streams
    This setup has been tested with USB video and RTSP streams
    """

    def __init__(self, stream_uri, output_filepath, frame_publisher, extension=".mkv", acodec="copy",
                 vcodec="copy", fps="15", video_bitrate="900k", stimeout=3000000, segment_time=300,
                 segment_format="matroska"):
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

        pre, ext = os.path.splitext(self.output_filepath)
        self.image_output_filepath = pre + ".bmp"
        self.segment_filelist = pre + "_video_list.txt"

        self._def_timeout_secs = 4.0
        self._def_extension = extension
        self._def_acodec = acodec
        self._def_vcodec = vcodec
        self._def_fps = fps
        self._def_bitrate = video_bitrate
        self._def_stimeout = stimeout  # stimeout in microseconds
        self._def_segment_time = segment_time
        self._def_min_video_size_bytes = 5000  # check if the output is at least some bytes before return success
        self._def_segment_fname_size = 3
        self._def_segment_format = segment_format

    def prepare_filepath_for_segment(self, filepath, segment_size):
        """
        Prepare a filepath for ffmpeg segments
        :param filepath:
        :param segment_size:
        :return:
        """
        fpath, ext = os.path.splitext(filepath)
        fpath_full = "{}-%0{}d{}".format(fpath, segment_size, ext)
        return fpath_full

    def start_recording(self):
        """
        Start recording the stream defined on self.stream_uri
        :return: boolean if the recording started sucessfully
        """

        # if we are dealing with a USB camera or RTSP stream we
        # treat them differently, for example remove the socket TCP I/O timeout (stimeout)
        parsed_uri = urlparse(self.stream_uri)
        if parsed_uri.scheme == "rtsp":
            stream_input = ffmpeg.input(self.stream_uri, nostdin=None, use_wallclock_as_timestamps=1,
                                        stimeout=self._def_stimeout, fflags="+genpts",
                                        rtsp_transport='tcp')  # stimeout in microsecondss
        else:
            stream_input = ffmpeg.input(self.stream_uri)

        # store the files in segments to prevent corruption
        segment_fpath = self.prepare_filepath_for_segment(self.output_filepath, self._def_segment_fname_size)

        # ffmpeg -use_wallclock_as_timestamps 1 -fflags +genpts -rtsp_transport tcp -stimeout 3000000
        # -i rtsp://admin:123456@192.168.3.22:554/stream0 -f segment -b:v 900k -an -flags +global_header -map 0
        # -map_metadata -1 -movflags +frag_keyframe+separate_moof+omit_tfhd_offset+empty_moov -reset_timestamps 1
        # -segment_format matroska
        # -segment_list /tmp/stored_streams/2.stream_192_168_3_22_554.2019-08-18.14.02.53_video_list.txt
        # -segment_list_type ffconcat -segment_time 20 -strict 2 -vcodec copy -use_wallclock_as_timestamps 1
        # -fflags +genpts /tmp/stored_streams/2.stream_192_168_3_22_554.2019-08-18.14.02.53-%03d.mkv -y

        output_arguments = [
            ("strict", 2),
            ("f", "segment"),
            ("map", 0),
            ("segment_time", self._def_segment_time),
            ("segment_format", self._def_segment_format),
            ("segment_list", self.segment_filelist),
            ("segment_list_type", "ffconcat"),
            ("vcodec", self._def_vcodec),
            ("video_bitrate", self._def_bitrate),
            ("flags", "+global_header"),
            ("reset_timestamps", 1),
            ("map_metadata", -1),
            ("use_wallclock_as_timestamps", 1),
            ("fflags", "+genpts"),
            ("movflags", "+frag_keyframe+separate_moof+omit_tfhd_offset+empty_moov"),
        ]

        # if there is no audio codec then use the an
        # flag to remove audio from the recorded stream
        if self._def_acodec != "":
            output_arguments.append(("acodec", self._def_acodec))
        else:
            output_arguments.append(("an", None))

        ffmpeg_output_streams = [
            ffmpeg.output(stream_input, segment_fpath, **OrderedDict(output_arguments))
        ]

        output_streams = ffmpeg.merge_outputs(*ffmpeg_output_streams)
        output_streams = ffmpeg.overwrite_output(output_streams)

        debug_command = ffmpeg.compile(output_streams)
        print("ffmpeg command: {}".format(' '.join(debug_command)))

        self.start_time = time.time()
        self.proc = (
            # ALERT: https://stackoverflow.com/questions/16523746/ffmpeg-hangs-when-run-in-background
            # clean the stderr / stdout regularly to prevent this process for freezing
            ffmpeg.run_async(output_streams, pipe_stdout=True, pipe_stderr=True, overwrite_output=True)
        )

        start_timeout = time.time()
        is_reached_size = False
        while self.proc.poll() is None and time.time() - start_timeout < self._def_timeout_secs and not is_reached_size:
            try:
                if os.path.getsize(self.get_last_segment_output_filepath()) > self._def_min_video_size_bytes:
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

        self.get_stdout_output()
        self.get_stderr_output()

        # send a ctrl+c signal to gracefully stop the process
        try:
            self.proc.send_signal(signal.SIGINT)
            self.wait_for_process_finished()
        except OSError as e:
            print "Error stopping ffmpeg for uri:{} {}".format(self.stream_uri, e)

        # force terminate the process
        if self.proc.poll() is None:
            print("Forcing terminate process: {}".format(self.proc.pid))
            self.proc.terminate()
            self.wait_for_process_finished()

            if self.proc.poll() is None:
                print("Sending SIGKILL to process: {}".format(self.proc.pid))
                os.kill(self.proc.pid, signal.SIGKILL)

        self.update_all_segments_duration()
        return True, ""

    def wait_for_process_finished(self):
        """
        Wait for process is finished, useful when stopping the recordings
        and trying to kill gracefully the process
        :return:
        """
        start_timeout = time.time()
        while self.proc.poll() is None and time.time() - start_timeout < self._def_timeout_secs:
            time.sleep(0.1)

    def get_all_video_segments(self):
        """
        Get a list of all video segments based on the self.output_filepath filename
        :return:
        """
        fpath, ext = os.path.splitext(self.output_filepath)
        return [x for x in glob.glob(os.path.join(fpath + "*"))]

    def get_last_segment_output_filepath(self):
        """
        Get the latest segment video file
        for example: the videos are saved in /tmp
            /tmp/vid-000.mkv
            /tmp/vid-003.mkv
            /tmp/vid-012.mkv

        This function will return the vid-012.mkv
        :return:
        """
        files = self.get_all_video_segments()

        convert = lambda text: int(text) if text.isdigit() else text.lower()
        alphanum_key = lambda key: [convert(c) for c in re.split('([0-9]+)', key)]
        files = sorted(files, key=alphanum_key)

        if len(files) > 0:
            return files[-1]
        else:
            return self.output_filepath

    def extract_metadata_from_stream(self):
        """
        Uses probe_video to extract the metadata of the video at the self.output_filepath file
        :return:
        """

        self.metadata = self.probe_video(self.stream_uri)
        return self.metadata

    def probe_video(self, filepath):
        """
        Uses ffmpeg.probe to extract the metadata of the video at the filepath file
        :return:
        """

        meta = None
        try:
            probe = ffmpeg.probe(filepath)
            meta = next((stream for stream in probe['streams'] if stream['codec_type'] == 'video'), None)
        except Exception as e:
            output = self._non_block_read(self.proc.stderr)
            print("Exception probing video metadata from {}. error:{}. out:{}".format(filepath, e, output))

        return meta

    def update_all_segments_duration(self):
        """
        Uses the metadata of the video to calculate the duration
        :return:
        """
        calculated_duration = 0

        files = self.get_all_video_segments()
        for f in files:
            metadata = self.probe_video(f)
            if metadata is None:
                print("Cannot get metadata from {}".format(f))
                continue

            # extract video duration and parse it to seconds
            # ffmpeg duration format is "00:04:04.199000000"
            self.duration = 0
            if 'tags' in metadata.keys() and 'DURATION' in metadata['tags'].keys():
                str_duration = metadata['tags']['DURATION']
                x = time.strptime(str_duration.split('.')[0], '%H:%M:%S')
                calculated_duration += int(datetime.timedelta(hours=x.tm_hour, minutes=x.tm_min, seconds=x.tm_sec) \
                                           .total_seconds())

        self.duration = calculated_duration

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

    def extract_fps_from_ffmpeg_output(self):
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

    def extract_frame_from_stream(self):
        """
        Extract the last frame of the stream beign recorded directly from the output file
        :return: np frame
        """

        if self.metadata is None:
            self.extract_metadata_from_stream()
            return None

        width = int(self.metadata['width'])
        height = int(self.metadata['height'])

        try:
            frame_cmd = ffmpeg.input(self.stream_uri).output('pipe:', vframes=1, format='rawvideo', pix_fmt='rgb24')

            out, _ = (
                frame_cmd.run(capture_stdout=True, capture_stderr=True)
            )

            video_stream = (
                np.frombuffer(out, np.uint8).reshape([-1, height, width, 3])
            )

            if video_stream.shape[0] == 1:
                RGB_img = cv2.cvtColor(video_stream[0], cv2.COLOR_BGR2RGB)
                return RGB_img
            else:
                return None
        except Exception as e:
            print("Exception extracting frame from {} error:{}".format(self.stream_uri, e))
            print("err:", _)
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

    def get_stdout_output(self):
        return self._non_block_read(self.proc.stdout)

    def get_stderr_output(self):
        return self._non_block_read(self.proc.stderr)

    def get_process(self):
        return self.proc


if __name__ == "__main__":
    pass
