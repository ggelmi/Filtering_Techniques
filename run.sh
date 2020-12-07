#!/bin/bash

octave  simulate

pushd /home/guuto/octave/Filtering_Techniques/kalmanFilter/build

cmake ..
make
./filter_exe
popd

octave --persist visualize

