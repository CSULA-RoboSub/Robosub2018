#!/bin/bash
v4l2-ctl -d /dev/video0 -c exposure_absolute=1500
v4l2-ctl -d /dev/video0 -p 60
