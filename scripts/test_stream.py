#!/usr/bin/env python
PKG = 'ros_service_store_rtsp'
import roslib
roslib.load_manifest(PKG)  # This line is not needed with Catkin.
import sys
import os
import rospy
import unittest
from store_rtsp_service import StoreStreamService
from ros_service_store_rtsp.srv import StoreRTSPRequest, StoreRTSPResponse
import rospkg
import time


class TestStoreStreamService(unittest.TestCase):

    def test_path_creation(self):
        store_service = StoreStreamService()

        paths = []
        for i in xrange(5):
            path, video_idx = store_service.get_output_filename("video.mp4")
            open(path, 'w').close()  # create_file in that folder
            paths.append(path)

        for p in paths:
            try:
                os.remove(p)
            except OSError:
                pass

        self.assertEquals(len(list(set(paths))), 5)

    def test_input_output(self):
        store_service = StoreStreamService()

        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('ros_service_store_rtsp')
        store_request = StoreRTSPRequest(True, "rtsp://wowzaec2demo.streamlock.net/vod/mp4:BigBuckBunny_115k.mov")
        store_service.store_response_callback(store_request)

        self.assertEquals(len(store_service.process_dict.keys()), 1)

        key = store_service.process_dict.keys()[0]
        proc = store_service.process_dict[key]
        output_filepath = proc.get_last_segment_output_filepath()

        self.assertTrue(os.path.isfile(output_filepath))

        store_service.stop_all_recordings()

        self.assertTrue(os.path.isfile(output_filepath))

        try:
            os.remove(output_filepath)
        except OSError:
            pass

        self.assertFalse(os.path.isfile(output_filepath))


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'test_stream', TestStoreStreamService) # , sysargs="--text"
