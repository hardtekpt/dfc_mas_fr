#!/usr/bin/env python3

import rospy
import sys
from os.path import dirname, abspath
sys.path.insert(0, dirname(dirname(dirname(abspath(__file__))))+"/crazyswarm/scripts")
from dfc_mas_fr.PublisherHelper import PublisherHelper
from dfc_mas_fr.srv import Commander, Map, MapResponse

from pycrazyswarm import *


if __name__ == '__main__':

    crazyflies_yaml = dirname(dirname(abspath(__file__)))+"/launch/crazyflies.yaml"
    swarm = Crazyswarm(crazyflies_yaml=crazyflies_yaml)

    rospy.init_node('publisher')
    ph = PublisherHelper(swarm)
    rospy.Service('publisher/map_handler', Map, ph.handle_map_srv)
    swarm.timeHelper.sleep(3)
    ph.run_algorithm()

