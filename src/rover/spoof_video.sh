#!/bin/bash

files="grb_1.mpg hale_bopp_1.mpg hst_1.mpg m84_1.mpg"
ffmpeg -re -i grb_1.mpg -f v4l2 /dev/video1 2>/dev/null 1>/dev/null &
ffmpeg -re -i hale_bopp_1.mpg -f v4l2 /dev/video2 2>/dev/null 1>/dev/null &
ffmpeg -re -i hst_1.mpg -f v4l2 /dev/video3 2>/dev/null 1>/dev/null &
ffmpeg -re -i m84_1.mpg -f v4l2 /dev/video4 2>/dev/null 1>/dev/null &

