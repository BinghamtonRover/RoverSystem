This is set up to work based on how OpenCV installed itself after I built it from source.
Make sure you download and build the `opencv_contrib` packages.

A 'calibration.yaml' file needs to be in this directory. This is specific to a given camera.

In order to create such a file, download and print this image: https://docs.opencv.org/master/pattern.png.
Then make sure the samples are built. Here are the commands I used to build/install OpenCV and its samples:
(make sure that you are in a `build` folder below the `opencv` root)
```
$ cmake -DOPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules/ -DBUILD_EXAMPLES=ON ../
$ make -j4
$ sudo make install
``` 
Now `bin/example_cpp_calibration` should be installed.

Calibrate the webcam by:
  1. Plugging it into your computer.
  2. Running the below command, replacing `<sqsz>` with the size of a square in meters.
  3. Hold the printed checkerboard in full view of the camera.
  4. Press 'g'.
  5. Wait for the screen to stop flashing.
  6. Press `escape` to exit.
```
bin/example_cpp_calibration -w=9 -h=6 -s=<sqsz> -o=calibration.yml
```

Then copy your `calibration.yml` file to this directory.
