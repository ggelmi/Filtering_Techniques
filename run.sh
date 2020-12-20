#!/bin/bash

pushd /home/guuto/octave/Filtering_Techniques/dataSimulator/build

cmake ..
make
./data_sim_exe

popd

octave --persist visualize

