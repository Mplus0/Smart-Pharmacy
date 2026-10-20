#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""ROS/tool entry point; implementation lives in pharmacy_iscc."""
import sys
from pharmacy_iscc.navigation.test_move_to_waypoint import main


if __name__ == "__main__":
    sys.exit(main())
