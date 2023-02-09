#include "my_turtle_controller/my_turtle_controller.h"

TurtleController::TurtleController():private_nh_("~"), nh_("")
{
    private_nh_.param("hz",hz_,{100});
    private_nh_.param("n",n_,{8});
    private_nh_.param("distance_target",distance_target_,{float(12.0/n_)});
    private_nh_.param("distance",distance_,{0.0});
    private_nh_.param("theta_target_base",theta_target_base_,{float(2.0*M_PI/n_)});
    private_nh_.param("turn_count",turn_count_,{0});
    pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel",1);
    sub_pose_ = nh_.subscribe("/turtle1/pose",10,&TurtleController::pose_callback,this);
}

void TurtleController::pose_callback(const turtlesim::Pose::ConstPtr &msg)
{
    current_pose_ = *msg; // nh_.subscribeで指定、current_pose_の更新
}

void TurtleController::set_pose(turtlesim::Pose &pose, float x=0.0, float y=0.0, float theta=0.0)
{
    pose.x     = x;
    pose.y     = y;
    pose.theta = theta;
}

int TurtleController::straight()
{
    cmd_vel_.linear.x  = 2.0;
    cmd_vel_.angular.z = 0.0;
    pub_cmd_vel_.publish(cmd_vel_); // publishを実行（turtleを動かす指令）
    return 1;
}

int TurtleController::turn()
{
    cmd_vel_.linear.x  = 0.0;
    cmd_vel_.angular.z = 1.7;
    pub_cmd_vel_.publish(cmd_vel_);
    return 2;
}

int TurtleController::stop()
{
    cmd_vel_.linear.x  = 0.0;
    cmd_vel_.angular.z = 0.0;
    pub_cmd_vel_.publish(cmd_vel_);
    return -1;
}

int TurtleController::move()
{
    int move_command = 0;

    if(distance_ < distance_target_)
    {
        move_command = straight(); // 直進
    }
    else if(can_turn() && turn_count_ < n_-1) // 直進より1回少ない
    {
        move_command = turn(); // 旋回
    }
    else if(turn_count_ == n_-1)
    {
        move_command = stop(); // 停止
    }
    else
    {
        old_pose_ = current_pose_; // 頂点のposeを記録
        turn_count_++;
    }

    return move_command;
}

float TurtleController::get_theta_target()
{
    float theta_target = theta_target_base_ * (turn_count_ + 1.0);
    if((-M_PI <= current_pose_.theta && current_pose_.theta< 0.0)||(2.0*M_PI<=theta_target))
    {
        theta_target -= 2.0*M_PI; // theta_targetとthetaの表示を合わせる
    }
    return theta_target;
}

bool TurtleController::can_turn()
{
    return current_pose_.theta < get_theta_target(); // 目標旋回角に達するまでTrue
}

void TurtleController::print_info(int move_command=0)
{
    switch(move_command)
    {
        case -1 : ROS_INFO("Move : Stop");     break;
        case  1 : ROS_INFO("Move : Straight"); break;
        case  2 : ROS_INFO("Move : Turn");     break;
        default: break;
    }
    ROS_INFO("turn_count : %d", turn_count_);
    ROS_INFO("current_pose");
    ROS_INFO_STREAM(current_pose_);
    ROS_INFO("old_pose");
    ROS_INFO_STREAM(old_pose_);
    ROS_INFO("distance          = %f", distance_);
    ROS_INFO("distance_target   = %f", distance_target_);
    ROS_INFO("distance_diff     = %f", distance_target_ - distance_);
    ROS_INFO("theta             = %f", current_pose_.theta);
    ROS_INFO("theta_target      = %f", get_theta_target());
    ROS_INFO("theta_target_diff = %f", get_theta_target() - current_pose_.theta);
    ROS_INFO("-------------------------------------\n\n\n");
}

void TurtleController::cal_distance()
{
    float dx = current_pose_.x - old_pose_.x;
    float dy = current_pose_.y - old_pose_.y;
    distance_ = sqrt(powf(dx, 2.0)+powf(dy, 2.0));
}

void TurtleController::process()
{
    ros::Rate loop_rate(hz_); // 周波数の設定
    set_pose(current_pose_, 5.544445, 5.544445); // 初期化
    old_pose_ = current_pose_; // 頂点のposeを記録
    int move_command = 0; // 実行動作を記録

    while(ros::ok())
    {
        cal_distance();
        move_command = move();
        print_info(move_command);

        ros::spinOnce();
        loop_rate.sleep(); // 周期が終わるまで待つ

        if(move_command == -1) break;
    }
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "my_turtle_controller"); // ノードの初期化
    TurtleController turtlecontroller1;
    turtlecontroller1.process();

    return 0;
}
