#include "image_moment/image_moment.h"

int flag_show = 0;

void ImageMoment::getIMUdata(const sensor_msgs::ImuConstPtr &msg)
{
    sensor_msgs::Imu pose_UAV_data;
    pose_UAV_data = *msg;
    qua_.x = pose_UAV_data.orientation.x;
    qua_.y = pose_UAV_data.orientation.y;
    qua_.z = pose_UAV_data.orientation.z;
    qua_.w = pose_UAV_data.orientation.w;
    eu1_.roll = atan2(2.0*(qua_.w*qua_.x + qua_.y*qua_.z), 1.0 - 2.0*(qua_.x*qua_.x + qua_.y*qua_.y));
    eu1_.pitch = asin(2*(qua_.w*qua_.y - qua_.z*qua_.x));
    eu1_.yaw = atan2(2.0*(qua_.w*qua_.z + qua_.y*qua_.x), 1.0 - 2.0*(qua_.z*qua_.z + qua_.y*qua_.y));
    
    if (flag_show == 1)
    {
        ROS_INFO("roll:%f",  eu1_.roll);
        ROS_INFO("pitch:%f", eu1_.pitch);
        ROS_INFO("yaw:%f",   eu1_.yaw);
    }

    //添加队列，用于保存最近的20个数据
    if (record_num < 20)
    {
        pose_data[0][record_num] = pose_UAV_data.header.stamp.toSec();
        pose_data[1][record_num] = eu1_.roll;
        pose_data[2][record_num] = eu1_.pitch;
        pose_data[3][record_num] = eu1_.yaw;
        ++record_num;
    }
    else
    {
        for (int i = 0; i < 19; ++i)
        {
            for (int j = 0; j < 4; ++j)
            {
                pose_data[j][i] = pose_data[j][i+1];
            }
        }
        pose_data[0][19] = pose_UAV_data.header.stamp.toSec();
        pose_data[1][19] = eu1_.roll;
        pose_data[2][19] = eu1_.pitch;
        pose_data[3][19] = eu1_.yaw;
    }
}

void ImageMoment::getImgdata(const image_moment::BoundingBoxes::ConstPtr& msg)
{
    targets = *msg;
    
    if (targets.bounding_boxes.empty())
    {
        ROS_INFO("No targets detected.");
        return;
    }
    // 选择置信度最高的检测目标
    auto max_prob_it = std::max_element(targets.bounding_boxes.begin(), 
    targets.bounding_boxes.end(),[](const auto& a, const auto& b)
    { 
        return a.probability < b.probability; 
    });
    target = *max_prob_it;
    
    if(target.probability > 0.5)
    {
        if(processing_type_ == "moment"  and record_num > 10)
        {
            // 检测获得的u和v
            corner[0][0] = target.xmin;
            corner[0][1] = target.ymin;
            corner[1][0] = target.xmax;
            corner[1][1] = target.ymin;
            corner[2][0] = target.xmax;
            corner[2][1] = target.ymax;
            corner[3][0] = target.xmin;
            corner[3][1] = target.ymax;
            
            // 获得对应时间下的无人机姿态角
            double min_time = 10.0;
            double delta_time = 0.0;
            int select_num = 0;
            for (int i = 0; i < record_num; ++i)
            {
                delta_time = abs(targets.header.stamp.toSec() - pose_data[0][i]);
                if (delta_time < min_time)
                {
                    min_time = delta_time;
                    select_num = i;
                }
            }
            // match_roll = pose_data[1][select_num];
            match_pitch = pose_data[2][select_num];
            match_yaw = pose_data[3][select_num];
            
            //获取四个角点在虚拟相机图像平面上的对应坐标
            obtain_Virtual_corner(corner, match_pitch, match_yaw, corner_virtual);

            double ug = 0.25 * (corner_virtual[0][0] + corner_virtual[1][0] + corner_virtual[2][0] + corner_virtual[3][0]);
            double vg = 0.25 * (corner_virtual[0][1] + corner_virtual[1][1] + corner_virtual[2][1] + corner_virtual[3][1]);
            double mu20 = pow(corner_virtual[0][0] - ug, 2) + pow(corner_virtual[1][0] - ug, 2) + pow(corner_virtual[2][0] - ug, 2) + pow(corner_virtual[3][0] - ug, 2);
            double mu02 = pow(corner_virtual[0][1] - vg, 2) + pow(corner_virtual[1][1] - vg, 2) + pow(corner_virtual[2][1] - vg, 2) + pow(corner_virtual[3][1] - vg, 2);
            double a = mu02 + mu20;
            bigTag_qh = sqrt(bigTag_a_star/a);
            bigTag_qx = ug * bigTag_qh/focal_length;
            bigTag_qy = vg * bigTag_qh/focal_length;

            double mu11 = (corner_virtual[0][0] - ug)*(corner_virtual[0][1] - vg) + (corner_virtual[1][0] - ug)*(corner_virtual[1][1] - vg) + (corner_virtual[2][0] - ug)*(corner_virtual[2][1] - vg) + (corner_virtual[3][0] - ug)*(corner_virtual[3][1] - vg);
            bigTag_qpsi = 0.5*atan(2*mu11/(mu20 - mu02));
            
            // bigTag_qpsi = atan2(corner_virtual[1][1] + corner_virtual[2][1] - 2*vg, corner_virtual[1][0] + corner_virtual[2][0] - 2*ug);
            // ROS_INFO("pos1:(%f, %f) ", corner_virtual[0][0], corner_virtual[0][1]);
            // ROS_INFO("pos2:(%f, %f) ", corner_virtual[1][0], corner_virtual[1][1]);
            // ROS_INFO("pos3:(%f, %f) ", corner_virtual[2][0], corner_virtual[2][1]);
            // ROS_INFO("pos4:(%f, %f)\n", corner_virtual[3][0], corner_virtual[3][1]);
            // ROS_INFO("36h11: qx = %.2f, qy = %.2f, qh = %.2f, qpsi = %.2f\n", bigTag_qx, bigTag_qy, bigTag_qh, bigTag_qpsi);
            moment.header.stamp = targets.header.stamp;
            moment.header.frame_id = "camera";
            moment.qx = bigTag_qx;
            moment.qy = bigTag_qy;
            moment.qh = bigTag_qh;
            moment.qpsi = bigTag_qpsi;
            moment_pub_.publish(moment);
            ROS_INFO("Ture");
        }
        else if(processing_type_ == "pixel")
        {
            // 像素中心计算
            center.x = ((target.xmin + target.xmax) / 2.0 - center_x) / focal_length;
            center.y = ((target.ymin + target.ymax) / 2.0 - center_y) / focal_length;
            center.z = 0;
            
            // 发布中心坐标
            pixel_pub_.publish(center);
            ROS_INFO("Pixel mode: Center(x=%.6f, y=%.6f)", center.x, center.y);
        }
    }
}

void ImageMoment::obtain_Virtual_corner(float corner[4][2], double pitch, double yaw, float corner_virtual[4][2])
{
    for (int i = 0; i < 4; ++i)
    {
        u_ture = corner[i][0] - center_x;
        v_ture = corner[i][1] - center_y;
        float M = -sin(pitch)*u_ture + cos(pitch)*focal_length;
        float Nu = focal_length*(cos(yaw)*cos(pitch)*u_ture - sin(yaw)*v_ture + cos(yaw)*sin(pitch)*focal_length);
        float Nv = focal_length*(sin(yaw)*cos(pitch)*u_ture + cos(yaw)*v_ture + sin(yaw)*sin(pitch)*focal_length);
        corner_virtual[i][0] = Nu/M;
        corner_virtual[i][1] = Nv/M;
    }
}
