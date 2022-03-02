#ifndef MY_TURTLE_CONTROLLER_H
#define MY_TURTLE_CONTROLLER_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>

class TurtleController
{
    public:
        TurtleController(); // デフォルトコンストラクタ
        void process();
    private:
        void pose_callback(const turtlesim::Pose::ConstPtr &msg);            // poseのコールバック関数
        void set_pose(turtlesim::Pose &pose ,float x, float y, float theta); // poseの設定
        void print_info(int move_command);                                   // 実行した動作と各種状態の表示

        int   straight();         // 直進
        int   turn();             // 旋回
        int   stop();             // 停止
        int   move();
        float get_theta_target(); // 目標旋回角を返す
        bool  can_turn();         // 目標旋回角に達するまでTrueを返す
        void  cal_distance();     // 直前の旋回地点と現在地との直線距離の算出


        int   hz_;                // ループ周波数
        int   n_;                 // N角形
        int   turn_count_;        // 方向転換の回数
        float distance_target_;   // 一辺のサイズ
        float distance_;          // 旋回位置（初期位置）と現在位置の直線距離
        float theta_target_base_; // 基準目標旋回角


        ros::NodeHandle nh_;           // ノードハンドル
        ros::NodeHandle private_nh_;   // プライベートノードハンドル
        ros::Subscriber sub_pose_;     // サブスクライバ（pose）
        ros::Publisher pub_cmd_vel_;   // パブリッシャ（速度指令）
        geometry_msgs::Twist cmd_vel_; // 速度指令
        turtlesim::Pose current_pose_; // 現在位置
        turtlesim::Pose old_pose_;     // 直前の旋回位置（初期位置）
};

#endif
