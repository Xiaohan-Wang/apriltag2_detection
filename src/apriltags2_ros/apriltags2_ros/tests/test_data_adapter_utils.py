#!/usr/bin/env python
import unittest, rosunit, rospy
import numpy as np
import math
import tf.transformations as tr
from apriltags2_ros_post_process.data_adapter_utils import *

class DataAdapterUtils(unittest.TestCase):


if __name__ == '__main__':
    rosunit.unitrun('apriltags2_ros', 'data_adapter_utils', DataAdapterUtils)
