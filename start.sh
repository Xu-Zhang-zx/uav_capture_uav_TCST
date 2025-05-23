# !/bin/bash
# tmux
cmd=$(which tmux)
session=uav_capture_uav
winName=mission

if [-z $cmd ]; then
	echo "You need to install tmux."
	exit 1
fi

$cmd kill-session -t $session

$cmd has -t $session

if [ $? != 0 ]; then
    $cmd new -s $session -d -n $winName
    
    # 分割顶部窗格（占30%高度）
    $cmd splitw -v -p 70 -t $winName
    
    # 进入下方大窗格并水平分割
    $cmd selectp -t 1
    $cmd splitw -h -p 50
    
    # 在左侧窗格垂直分割
    $cmd selectp -t 1
    $cmd splitw -v -p 50
    
    # 在右侧窗格垂直分割
    $cmd selectp -t 3
    $cmd splitw -v -p 50
    
    # 返回顶部窗格（pane 0）
    $cmd selectp -t 0
fi

$cmd selectp -t 0
$cmd send-keys "roslaunch px4 uav_capture_uav.launch" C-m
$cmd selectp -t 1
$cmd send-keys "sleep 10;cd /home/xu/uav_capture_uav/scripts/uav_communication/;bash multi_vehicle_communication.sh" C-m
$cmd selectp -t 3
$cmd send-keys "sleep 15;cd /home/xu/uav_capture_uav;source devel/setup.bash;conda activate yolov11;roslaunch yolov11_ros yolo_v11.launch" C-m
$cmd selectp -t 2
$cmd send-keys "sleep 10;cd ~/XTDrone/control/keyboard;python3.8 multirotor_keyboard_control.py typhoon_h480 1 vel" C-m
$cmd selectp -t 4
$cmd send-keys "sleep 10;cd ~/XTDrone/control/keyboard;python3.8 multirotor_keyboard_control.py iris 1 vel" C-m

$cmd att -t $session
