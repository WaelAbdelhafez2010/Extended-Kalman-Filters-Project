#! /bin/bash
sudo rm /home/wael/.config/unity3d/Udacity/self_driving_car_nanodegree_program/Player.log
cmake ..
make
./ExtendedKF
