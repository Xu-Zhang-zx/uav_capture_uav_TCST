#!/bin/bash
iris_num=1
typhoon_h480_num=1

vehicle_num=0
while(( $vehicle_num < iris_num)) 
do
    python3.8 multirotor_communication.py iris $vehicle_num&
    let "vehicle_num++"
done

vehicle_num=0
while(( $vehicle_num < typhoon_h480_num)) 
do
    python3.8 multirotor_communication.py typhoon_h480 $vehicle_num&
    let "vehicle_num++"
done

