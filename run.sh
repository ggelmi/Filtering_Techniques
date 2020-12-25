#!/bin/bash
: <<'END'
pushd matlab_simulator/linear_simulator
octave kf_simulate 
popd

pushd /home/guuto/octave/Filtering_Techniques/mainApp/build

cmake ..
make
./almis_app_exe

popd

pushd matlab_simulator/linear_simulator
octave --persist kf_visualize
popd
END
pushd matlab_simulator/non_linear_simulator
octave ekf_simulate.m
popd
pushd /home/guuto/octave/Filtering_Techniques/mainApp/build

cmake ..
make
./almis_app_exe

popd
pushd matlab_simulator/non_linear_simulator
octave --persist ekf_visualize.m
popd
