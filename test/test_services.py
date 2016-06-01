#!/usr/bin/env python
# -*- python -*-

import sys
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
        rospy.wait_for_service("/Dump")
        rospy.wait_for_service("/Config")

        self.dump_srv = rospy.ServiceProxy("/Dump", Dump)
        self.config_srv = rospy.ServiceProxy("/Config", Config)

        dump_resp = self.dump_srv()
        json = simplejson.loads(dump_resp.config)
        idx = int(json["o3d3xx"]["Device"]["ActiveApplication"])
        self.assertTrue(idx > 0)

        params = [(idx, 5, "upto30m_moderate"),
                  (idx, 10, "under5m_low"),
                  (idx, 5, "upto30m_moderate")]
        for param_set in params:
            self._dump_config(*param_set)

    def _dump_config(self, idx, frame_rate, im_type):
        dump_resp = self.dump_srv()
        json = simplejson.loads(dump_resp.config)

        real_idx = 0
        for app in json["o3d3xx"]["Apps"]:
            if int(app["Index"]) == idx:
                break
            else:
                real_idx += 1

        json["o3d3xx"]["Apps"][real_idx]["Imager"]["FrameRate"] = \
          str(frame_rate)
        json["o3d3xx"]["Apps"][real_idx]["Imager"]["Type"] = str(im_type)

        config_resp = self.config_srv(simplejson.dumps(json))
        self.assertTrue(config_resp.status == 0)

        dump_resp = self.dump_srv()
        json = simplejson.loads(dump_resp.config)

        fr = int(json["o3d3xx"]["Apps"][real_idx]["Imager"]["FrameRate"])
        it = json["o3d3xx"]["Apps"][real_idx]["Imager"]["Type"]
        self.assertTrue(fr == frame_rate)
        self.assertTrue(it == im_type)

def main():
    rostest.rosrun('o3d3xx', 'test_services', TestServices, sys.argv)
    return 0

if __name__ == '__main__':
    sys.exit(main())
