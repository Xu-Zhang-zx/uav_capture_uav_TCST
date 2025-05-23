#include <intruder_uav/control.h>
#include <cmath>
#include <unistd.h>

template<typename T>
const T& clamp(const T& value, const T& low, const T& high) {
    return (value < low) ? low : (value > high) ? high : value;
}

// 初始化用户界面
void IntruderControl::init_ui()
{
    initscr();
    cbreak();              // 禁用行缓冲
    noecho();              // 不显示输入字符
    keypad(stdscr, TRUE);  // 启用功能键
    timeout(0);            // 非阻塞输入
    
    // 创建状态显示窗口
    status_win_ = newwin(LINES, COLS, 0, 0);
    box(status_win_, 0, 0);
    mvwprintw(status_win_, 1, 1, "Intruder UAV Control System");
    wrefresh(status_win_);

    start_color();
    init_pair(1, COLOR_RED, COLOR_BLACK);     // 返航状态 
    init_pair(2, COLOR_GREEN, COLOR_BLACK);   // 航点导航
    init_pair(3, COLOR_BLUE, COLOR_BLACK);    // 悬停状态
    init_pair(4, COLOR_WHITE, COLOR_BLACK);   // 板外起飞
    init_pair(5, COLOR_YELLOW, COLOR_BLACK);  // 待机状态
}

// 更新用户界面
void IntruderControl::update_ui()
{
    // 清空内容区域
    wclear(status_win_);
    box(status_win_, 0, 0);
    
    // 显示状态信息
    mvwprintw(status_win_, 2, 2, "Current State: ");
    switch(current_state_)
    {
        case IDLE: 
            wattron(status_win_, COLOR_PAIR(5));
            wprintw(status_win_, "IDLE"); 
            wattroff(status_win_, COLOR_PAIR(5)); 
            break;
        case OFFBOARD:
            wattron(status_win_, COLOR_PAIR(4));
            wprintw(status_win_, "OFFBOARD"); 
            wattroff(status_win_, COLOR_PAIR(4)); 
            break;
        case HOVERING:
            wattron(status_win_, COLOR_PAIR(3));
            wprintw(status_win_, "HOVERING"); 
            wattroff(status_win_, COLOR_PAIR(3));
            break;        
        case TAKEOFF: 
            wprintw(status_win_, "TAKEOFF"); 
            break;
        case FOLLOWING_WAYPOINTS: 
            wattron(status_win_, COLOR_PAIR(2));
            wprintw(status_win_, "WAYPOINT NAVIGATION"); 
            wattroff(status_win_, COLOR_PAIR(2)); 
            break;
        case RETURNING: 
            wattron(status_win_, COLOR_PAIR(1)); 
            wprintw(status_win_, "RETURNING"); 
            wattroff(status_win_, COLOR_PAIR(1)); 
            break;
    }
    
    // 显示当前位置
    mvwprintw(status_win_, 4, 2, "Position (x,y,z):");
    mvwprintw(status_win_, 5, 4, "%.2f, %.2f, %.2f", 
             local_pose.pose.position.x,
             local_pose.pose.position.y,
             local_pose.pose.position.z);
    
    // 显示控制提示
    mvwprintw(status_win_, 7, 2, "Commands:");
    mvwprintw(status_win_, 8, 4, "[B] Offboard  [T] Takeoff [S] Hover [R] Return [Q] Quit");
    
    int msg_line = LINES - 7;
    int min_msg_line = 9;
    int max_display_lines = std::min(MAX_MSG_LINES, (LINES - 5 - min_msg_line));
    int lines_printed = 0;
    
    for (auto it = status_messages.rbegin(); it != status_messages.rend(); ++it) 
    {
        // 截断消息以避免自动换行
        std::string truncated_msg = it->substr(0, COLS - 3);
        mvwprintw(status_win_, msg_line--, 2, "> %s", truncated_msg.c_str());
        lines_printed++;
    
        // 边界检查：不超过可显示区域或最大行数
        if (msg_line <= min_msg_line || lines_printed >= max_display_lines) 
        {
            break;
        }
    }

    wrefresh(status_win_);
    doupdate();

    // 处理键盘输入
    int ch = getch();
    if(ch != ERR) process_input(ch);
}

// 关闭用户界面
void IntruderControl::shutdown_ui()
{
    delwin(status_win_);
    endwin();
}

// 处理键盘输入
void IntruderControl::process_input(int ch)
{
    std::stringstream cmd;
    switch(toupper(ch))
    {
        case 'B':  // 板外模式切换 
            if(current_state_ == IDLE)
            {
                cmd << "OFFBOARD";
                key.data  = cmd.str(); 
                cmd_pub.publish(key); 
                status_messages.push_back(std::string("Offboard mode activation in progress..."));
                if(status_messages.size()  > MAX_MSG_LINES) 
                {
                    status_messages.pop_front(); 
                }
                current_state_ = OFFBOARD;
            }
            break;
            
        case 'T':  // 解锁起飞 
            if(current_state_ == OFFBOARD)
            {
                cmd << "ARM";
                key.data  = cmd.str(); 
                cmd_pub.publish(key); 
                current_state_ = TAKEOFF;
                status_messages.push_back(std::string("Unlock command sent."));
                if(status_messages.size()  > MAX_MSG_LINES) 
                {
                    status_messages.pop_front(); 
                }
            }
            break;
            
        case 'S':  // 悬停控制 
            if((current_state_ == FOLLOWING_WAYPOINTS) && !flag_hover)
            { 
                // 起飞后允许悬停 
                current_state_ = HOVERING;
                flag_hover = true;
                status_messages.push_back(std::string("Enter hover mode."));
                if(status_messages.size()  > MAX_MSG_LINES) 
                {
                    status_messages.pop_front(); 
                }
                break;
            }
            if ((current_state_ == HOVERING) && flag_hover)
            {
                flag_hover = false;
                current_state_ = FOLLOWING_WAYPOINTS;
                break;
            }
            break;
            
        case 'R':  // 返航指令 
            if(current_state_ != IDLE)
            {
                cmd << "AUTO.RTL";
                key.data  = cmd.str(); 
                cmd_pub.publish(key); 
                current_state_ = RETURNING;
                status_messages.push_back(std::string("Return instruction activated."));
                if(status_messages.size()  > MAX_MSG_LINES) 
                {
                    status_messages.pop_front(); 
                }
            }
            break;
            
        case 'Q':  // 紧急退出 
            shutdown_ui();
            ros::shutdown();
            exit(0);
    }
}

void IntruderControl::velocityControl()
{
    switch(current_state_)
    {
        case TAKEOFF:
        {
            if(local_pose.pose.position.z  < 4.95)
            {
                cmd_vel_enu.linear.z  = 0.5;
                vel_pub_.publish(cmd_vel_enu);
            } 
            else 
            {
                current_state_ = ESCAPE;
                status_messages.push_back(std::string("Reached cruising altitude of 3m."));
                if(status_messages.size()  > MAX_MSG_LINES) 
                {
                    status_messages.pop_front(); 
                }
            }
            break;
        }
        case HOVERING:
        {
            cmd_vel_enu.linear.x = 0;
            cmd_vel_enu.linear.y = 0;
            cmd_vel_enu.linear.z = 0;
            
            vel_pub_.publish(cmd_vel_enu);
            break;
        }
        case ESCAPE:
        {
            // escape_strategy:
            //    z_escape;
            //    sin_escape;
            //    circle_escape;
            std::srand(std::time(nullptr));
            int random_num = std::rand() % 3 + 1;
            if(random_num == 1) 
            {
                current_state_ = z_escape;
            }
            else if(random_num == 2) 
            {
                current_state_ = sin_escape;
            }
            else if(random_num == 3) 
            {
                current_state_ = circle_escape;
            }
            break;
        }
        case z_escape:
        {
            cur_x = local_pose.pose.position.x;
            cur_y = local_pose.pose.position.y;
            cur_z = local_pose.pose.position.z;

            waypoints_.clear();
            set_point.y = cur_y + 3.0;
            set_point.z = cur_z;
            waypoints_.push_back(set_point);
            set_point.y = cur_y;
            set_point.z = cur_z;
            waypoints_.push_back(set_point);
            set_point.y = cur_y - 2.0;
            set_point.z = cur_z;
            waypoints_.push_back(set_point);
            set_point.y = cur_y;
            set_point.z = cur_z;
            waypoints_.push_back(set_point);
            current_waypoint_index_ = 0;
            current_state_ = FOLLOWING_WAYPOINTS;
 
            break;
        }
        case sin_escape:
        {
            cur_x = local_pose.pose.position.x;
            cur_y = local_pose.pose.position.y;
            cur_z = local_pose.pose.position.z;

            waypoints_.clear();
            set_point.y = cur_y;
            set_point.z = cur_z + 3.0;
            waypoints_.push_back(set_point);
            set_point.y = cur_y;
            set_point.z = cur_z;
            waypoints_.push_back(set_point);
            set_point.y = cur_y;
            set_point.z = cur_z - 1.5;
            waypoints_.push_back(set_point);
            set_point.y = cur_y;
            set_point.z = cur_z;
            waypoints_.push_back(set_point);
            current_waypoint_index_ = 0;
            current_state_ = FOLLOWING_WAYPOINTS;
            
            break;
        }
        case circle_escape:
        {
            cur_x = local_pose.pose.position.x;
            cur_y = local_pose.pose.position.y;
            cur_z = local_pose.pose.position.z;

            waypoints_.clear();
            for (int i = 0; i < 8; i++)
            {
                double angle = i * PI / 4.0;
                set_point.y = cur_y + 2.0 * cos(angle);
                set_point.z = cur_z + 2.0 * sin(angle);
                waypoints_.push_back(set_point);
            }
            current_waypoint_index_ = 0;
            current_state_ = FOLLOWING_WAYPOINTS;
            
            break;
        }
        case FOLLOWING_WAYPOINTS:
        {

            if(!waypoints_.empty() && current_waypoint_index_ < waypoints_.size())
            {
                geometry_msgs::Point target = waypoints_[current_waypoint_index_];
                double dy = target.y - local_pose.pose.position.y;
                double dz = target.z - local_pose.pose.position.z;

                // 分别计算y和z方向的误差
                double dist_y = fabs(dy);
                double dist_z = fabs(dz);

                cal_pid(dy, dz);

                if(dist_y < 0.3 && dist_z < 0.2)
                {
                    current_waypoint_index_ ++;
                    if(current_waypoint_index_ >= waypoints_.size())
                    {
                        current_waypoint_index_ = 0;
                        waypoints_.clear();
                        status_messages.push_back(std::string("Reached final waypoint. Looping..."));
                        current_state_ = ESCAPE;
                    }
                    else
                    {
                        // 使用字符串流格式化消息
                        std::ostringstream msg;
                        msg << "Reached waypoint " << (current_waypoint_index_) << ". Proceeding to next.";
                        status_messages.push_back(msg.str());
                    }
                    if(status_messages.size()  > MAX_MSG_LINES)
                    {
                        status_messages.pop_front(); 
                    }
                }
            }
            break;
        }
        case RETURNING:
        {
            geometry_msgs::Point home;
            home.x = home.y = home.z = 0;
            double ddx = home.x - local_pose.pose.position.x; 
            double ddy = home.y - local_pose.pose.position.y; 
            double ddz = home.z - local_pose.pose.position.z; 
            cal_pid(ddy,ddz,ddx);
            
            if(sqrt(ddx*ddx + ddy*ddy + ddz*ddz) < 0.5)
            {
                current_state_ = IDLE;
            }
            break;
        }
    }
}

void IntruderControl::poseSubscribe(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    local_pose = *msg;
}

void IntruderControl::cal_pid(double dy, double dz, double dx)
{
    // X方向保持固定速度
    cmd_vel_enu.linear.x = MAX_SPEED;
    
    // Y方向正常PID控制
    cmd_vel_enu.linear.y = Kp * dy;
    
    // Z方向使用更高的增益和重力补偿
    double Kp_z = 1.5 * Kp;  // 高度控制增益更大
    double gravity_comp = 0.05;  // 小的重力补偿项
    cmd_vel_enu.linear.z = Kp_z * dz + gravity_comp;
    
    // 速度限幅
    cmd_vel_enu.linear.y = clamp(cmd_vel_enu.linear.y, -MAX_SPEED, MAX_SPEED);
    cmd_vel_enu.linear.z = clamp(cmd_vel_enu.linear.z, -MAX_SPEED, MAX_SPEED);

    vel_pub_.publish(cmd_vel_enu);
}