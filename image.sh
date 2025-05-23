# !/bin/bash
# tmux
cmd=$(which tmux)
session=yolov11
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
    # $cmd selectp -t 1
    # $cmd splitw -v -p 40
    
    # # 在下方第三个窗格水平分割
    # $cmd selectp -t 2
    # $cmd splitw -v -p 50
    
    # 返回顶部窗格（pane 0）
    $cmd selectp -t 0
fi

$cmd selectp -t 0
$cmd send-keys "cd /home/xu/uav_capture_uav;source devel/setup.bash;rosrun image_moment image_moment_node _type:=pixel" C-m
$cmd selectp -t 1
$cmd send-keys "cd /home/xu/uav_capture_uav;source devel/setup.bash;rosrun intruder_uav intruder_uav_node" C-m
$cmd selectp -t 2
# $cmd send-keys "" C-m
# $cmd selectp -t 4
# $cmd send-keys "sleep 10;cd ~/XTDrone/control/keyboard;python3.8 multirotor_keyboard_control.py iris 1 vel" C-m
# $cmd selectp -t 2
# $cmd send-keys "sleep 10;cd ~/XTDrone/control/keyboard;python3.8 multirotor_keyboard_control.py typhoon_h480 1 vel" C-m

$cmd att -t $session
