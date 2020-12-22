#!/bin/bash

octave simulate 

pushd /home/guuto/octave/Filtering_Techniques/mainApp/build

cmake ..
make
./almis_app_exe

popd

octave --persist visualize

