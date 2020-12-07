#!/bin/bash

octave  main

pushd /home/guuto/octave/Filtering_Techniques/kalmanFilter/build

cmake ..
make
./filter_exe
popd

octave --persist simulate

