#!/usr/bin/env python
# -*- python -*-

import sys
import time
import unittest
import rospy
import rostest
import simplejson
from o3d3xx.srv import Dump, Config

class TestServices(unittest.TestCase):

    def __init__(self, *args):
        super(TestServices, self).__init__(*args)

    def test_services(self):
        rospy.init_node('test_services')
        rospy.wait_for_service("/Dump", 5.0)
        rospy.wait_for_service("/Config", 1.0)

        self.dump_srv = rospy.ServiceProxy("/Dump", Dump)
        self.config_srv = rospy.ServiceProxy("/Config", Config)

        dump_resp = self.dump_srv()
        json = simplejson.loads(dump_resp.config)
        idx = int(json["o3d3xx"]["Device"]["ActiveApplication"])

        params = [(idx, 5, "upto30m_moderate"),
                  (idx, 10, "under5m_low"),
                  (idx, 5, "upto30m_moderate")]
        for param_set in params:
            self._dump_config(*param_set)
            time.sleep(1.0)

    def _dump_config(self, idx, frame_rate, im_type):
        dump_resp = self.dump_srv()
        json = simplejson.loads(dump_resp.config)

        self.assertTrue(idx > 0)
        json["o3d3xx"]["Apps"][idx-1]["Imager"]["FrameRate"] = str(frame_rate)
        json["o3d3xx"]["Apps"][idx-1]["Imager"]["Type"] = str(im_type)

        config_resp = self.config_srv(simplejson.dumps(json))
        self.assertTrue(config_resp.status == 0)

        dump_resp = self.dump_srv()
        json = simplejson.loads(dump_resp.config)

        fr = int(json["o3d3xx"]["Apps"][idx-1]["Imager"]["FrameRate"])
        it = json["o3d3xx"]["Apps"][idx-1]["Imager"]["Type"]
        self.assertTrue(fr == frame_rate)
        self.assertTrue(it == im_type)

def main():
    rostest.rosrun('o3d3xx', 'test_services', TestServices, sys.argv)
    return 0

if __name__ == '__main__':
    sys.exit(main())
