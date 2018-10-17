#!/bin/bash

file="output.mkv"
ffmpeg -re -i $file -f v4l2 /dev/video1 2>/dev/null 1>/dev/null &
ffmpeg -re -i $file -f v4l2 /dev/video2 2>/dev/null 1>/dev/null &
ffmpeg -re -i $file -f v4l2 /dev/video3 2>/dev/null 1>/dev/null &
ffmpeg -re -i $file -f v4l2 /dev/video4 2>/dev/null 1>/dev/null &
