/*
功能：使小车跟踪路标点
输入：way_point,  小车模型， 2D地图
输出：差速，输出到下位机
*/

// #include "TRACTKING_TRAJ/Pure_Pursuit.h"
#include "tianracer_navigation/Pure_Pursuit.h"
using namespace std;

PurePursuit::PurePursuit() {
    // 创建私有的节点句柄
    ros::NodeHandle pravite_nh("~");
    //Car parameter
    pravite_nh.param("base_shape_L", base_shape_L, 0.49); // 机器人轴距
    pravite_nh.param("reference_v", reference_v, 0.3);// 目标速度
    pravite_nh.param("Lfw", Lfw, 0.5); // 前视距离

    //Controller parameter
    pravite_nh.param("controller_freq", controller_freq_, 50);   // 控制频率
    pravite_nh.param("goal_radius", goal_radius, 0.2); // 目标容忍度

    //Publishers and Subscribers
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/path_marker", 10);
    twist_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1); 
    ackermann_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("/drive", 1);  
    path_sub_ = nh_.subscribe("/global_plan", 1, &PurePursuit::pathCallback, this);
    timer1 = nh_.createTimer(ros::Duration((1.0) / controller_freq_), &PurePursuit::controlLoopCallback, this); // 发布控制
    odom_sub_ = nh_.subscribe("/odom_link", 1, &PurePursuit::mapCallback, this);
    //Init variables
    foundForwardPt_ = false;
    goal_received_ = false;
    goal_reached_ = false;
    velocity = 0.0;
    steering_ = 0.0;

//---------
    twist_cmd_ = geometry_msgs::Twist();
    // 新添
    ackermann_cmd_ = ackermann_msgs::AckermannDriveStamped();
    initMarker();

}


PurePursuit::~PurePursuit(){};

//这一部分是设置map的基本信息
void PurePursuit::initMarker() {
    points_.header.frame_id = line_strip_.header.frame_id = goal_circle_.header.frame_id = "odom";
    points_.ns = line_strip_.ns = goal_circle_.ns = "Markers";
    points_.action = line_strip_.action = goal_circle_.action = visualization_msgs::Marker::ADD;
    points_.pose.orientation.w = line_strip_.pose.orientation.w = goal_circle_.pose.orientation.w = 1.0;
    points_.id = 0;
    line_strip_.id = 1;
    goal_circle_.id = 2;
    points_.type = visualization_msgs::Marker::POINTS;
    line_strip_.type = visualization_msgs::Marker::LINE_STRIP;
    goal_circle_.type = visualization_msgs::Marker::CYLINDER;
    points_.scale.x = 0.2;
    points_.scale.y = 0.2;
    line_strip_.scale.x = 0.1;
    goal_circle_.scale.x = goal_radius;
    goal_circle_.scale.y = goal_radius;
    goal_circle_.scale.z = 0.1;
    points_.color.g = 1.0f;
    points_.color.a = 1.0;
    line_strip_.color.b = 1.0;
    line_strip_.color.a = 1.0;
    goal_circle_.color.r = 1.0;
    goal_circle_.color.g = 1.0;
    goal_circle_.color.b = 0.0;
    goal_circle_.color.a = 0.5;
}


// /*!
//  *
//  * @param  odomMsg odom传感器数据
//  * @author 测试
//  * @author odom数据会出现漂移导致不准，可尝试使用tf变换 计算出base_link在map下的坐标
//  */
// void PurePursuit::odomCallback(const nav_msgs::Odometry::ConstPtr &odomMsg) {
//     this->odom_ = *odomMsg;
//     if(this->goal_received_)
//     {
//         double car2goal_x = this->goal_pos_.x - odomMsg->pose.pose.position.x;
//         double car2goal_y = this->goal_pos_.y - odomMsg->pose.pose.position.y;
//         double dist2goal = sqrt(car2goal_x*car2goal_x + car2goal_y*car2goal_y);
//     }
// }

void PurePursuit::mapCallback(const nav_msgs::Odometry::ConstPtr &mapMsg) {
    this->map_ = *mapMsg;
    if(this->goal_received_)
    {
        double car2goal_x = this->goal_pos_.x - mapMsg->pose.pose.position.x;
        double car2goal_y = this->goal_pos_.y - mapMsg->pose.pose.position.y;
        double dist2goal = sqrt(car2goal_x*car2goal_x + car2goal_y*car2goal_y);
        /// 和move_base一起使用时这段注释解除
	// if(dist2goal < this->goal_radius)
    //   {
    //        this->goal_reached_ = true;
    //        this->goal_received_ = false;
    //        ROS_INFO("Goal Reached !");
    //    }
    }
}

/*!
 *
 * @param pathMsg 全局路径
 * TODO importion 后期会使用局部路径
 */
void PurePursuit::pathCallback(const nav_msgs::Path::ConstPtr &pathMsg) {
    this->map_path_ = *pathMsg;
    goal_received_ = true;
    goal_reached_  = false;
}
// /*!
//  *
//  * @param goalMsg
//  */
void PurePursuit::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& goalMsg)
{
    this->goal_pos_ = goalMsg->pose.position;
    try
    {
        geometry_msgs::PoseStamped map_goal;
        //tf_listener_.transformPose("odom", ros::Time(0) , *goalMsg, "map" ,odom_goal);
        map_goal_pos_ = map_goal.pose.position;
        goal_received_ = true;
        goal_reached_ = false;
        /*Draw Goal on RVIZ*/
        goal_circle_.pose = map_goal.pose;
        marker_pub_.publish(goal_circle_);
    }
    catch(tf::TransformException &ex)
    {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
}


//geometry_msgs::Point& car_pos:包含位置信息无角度信息
//geometry_msgs::Pose& carPose:包含位置信息和角度信息
//判断找到前瞻点是否在前视距离之外
bool PurePursuit::isWayPtAwayFromLfwDist(const geometry_msgs::Point& wayPt, const geometry_msgs::Point& car_pos)
{
    double dx = wayPt.x - car_pos.x;
    double dy = wayPt.y - car_pos.y;
    double dist = sqrt(dx*dx + dy*dy);

    if(dist < Lfw)
        return false;
    else if(dist >= Lfw)
        return true;
}


//判断得到的前瞻点在当前位置的前面还是后面
bool PurePursuit::isForwardWayPt(const geometry_msgs::Point& wayPt, const geometry_msgs::Pose& carPose)
{
    float car2wayPt_x = wayPt.x - carPose.position.x;
    float car2wayPt_y = wayPt.y - carPose.position.y;
    double car_theta =  tf::getYaw(carPose.orientation);

    float car_car2wayPt_x = cos(car_theta)*car2wayPt_x + sin(car_theta)*car2wayPt_y;

    if(car_car2wayPt_x >0) /*is Forward WayPt*/
        return true;
    else
        return false;
}

//计算路径点到小车map坐标系下的距离的平方
double PurePursuit::PointDistanceSquare(geometry_msgs::PoseStamped& wayPt,const geometry_msgs::Point& car_pos){
    double dx = wayPt.pose.position.x - car_pos.x;
    double dy = wayPt.pose.position.y - car_pos.y;
    return dx * dx + dy * dy;
}

//子函数：用来确定当前数组中的最小值
int PurePursuit::minIndex(const geometry_msgs::Point& carPose){
    int index_min = 0;
    int d_min = INT16_MAX;
    for(int i =0; i< map_path_.poses.size(); i++)
    {
        geometry_msgs::PoseStamped map_path_pose = map_path_.poses[i];
        double d_temp = PointDistanceSquare(map_path_pose,carPose);
        if (d_temp < d_min) {
            d_min = d_temp;
            index_min = i;
        }
    }
    return index_min;
}


//计算前瞻点到小车的绝对距离（在map坐标系下）
geometry_msgs::Point PurePursuit::get_map_car2WayPtVec(const geometry_msgs::Pose& carPose)
{
    geometry_msgs::Point carPose_pos = carPose.position;//map坐标系下小车此时的位置信息（只有坐标点）
    double carPose_yaw = tf::getYaw(carPose.orientation);//四元数变换得到小车此时的偏航角
    geometry_msgs::Point forwardPt;//声明变量forwardpt为前瞻点的map坐标系下的位置
   geometry_msgs::Point map_car2WayPtVec;
    foundForwardPt_ = false;

    if(!goal_reached_){
        //寻找离小车最近的路径点
        int index = minIndex(carPose_pos);
        for(int i =index; i< map_path_.poses.size(); i++)
        {
            geometry_msgs::PoseStamped map_path_pose = map_path_.poses[i];
            //geometry_msgs::PoseStamped odom_path_pose;
            geometry_msgs::PoseStamped car_pose;
            try
            {
                //tf_listener_.transformPose("odom", ros::Time(0) , map_path_pose, "map" ,odom_path_pose);
                geometry_msgs::Point map_path_wayPt = car_pose.pose.position;

                bool _isForwardWayPt = isForwardWayPt(map_path_wayPt,carPose);

                if(_isForwardWayPt)
                {
                    bool _isWayPtAwayFromLfwDist = isWayPtAwayFromLfwDist(map_path_wayPt,carPose_pos);
                    if(_isWayPtAwayFromLfwDist)
                    {
                        forwardPt = map_path_wayPt;
                        foundForwardPt_ = true;
                        break;
                    }
                }
            }
            catch(tf::TransformException &ex)
            {
                ROS_ERROR("%s",ex.what());
                ros::Duration(1.0).sleep();
            }
        }

    }
    else if(goal_reached_)
    {
        forwardPt = map_goal_pos_;
        foundForwardPt_ = false;
	    velocity = 0.0;
	    steering_ = 0.0;
    }

    /*Visualized Target Point on RVIZ*/
    /*Clear former target point Marker*/
    points_.points.clear();
    line_strip_.points.clear();

    if(foundForwardPt_ && !goal_reached_)
    {
        points_.points.push_back(carPose_pos);
        points_.points.push_back(forwardPt);
        line_strip_.points.push_back(carPose_pos);
        line_strip_.points.push_back(forwardPt);
    }

    marker_pub_.publish(points_);
    marker_pub_.publish(line_strip_);

    // map坐标系下前瞻点与小车的距离
    map_car2WayPtVec.x = cos(carPose_yaw)*(forwardPt.x - carPose_pos.x) + sin(carPose_yaw)*(forwardPt.y - carPose_pos.y);
    map_car2WayPtVec.y = -sin(carPose_yaw)*(forwardPt.x - carPose_pos.x) + cos(carPose_yaw)*(forwardPt.y - carPose_pos.y);
    return map_car2WayPtVec;
}




//获得航向角（阿尔法）
double PurePursuit::getEta(const geometry_msgs::Pose& carPose)
{
    geometry_msgs::Point map_car2WayPtVec = get_map_car2WayPtVec(carPose);
    return atan2(map_car2WayPtVec.y,map_car2WayPtVec.x);
}

//根据获得的航向角和前视距离等数据计算转向角度（德尔塔），以便后续转换为角速度
double PurePursuit::getSteering(double eta)
{
    return atan2(2*(this->base_shape_L*sin(eta)),(this->Lfw));
}

void PurePursuit::controlLoopCallback(const ros::TimerEvent&)
{
    geometry_msgs::Pose carPose = this->map_.pose.pose;
    // geometry_msgs::Twist carVel = this->map_.twist.twist;
    // double eta = getEta(carPose);
    if(this->goal_received_)
    {

        double eta = getEta(carPose);
//        cout<<eta<<endl;
        if(foundForwardPt_)
        {
            //this->steering_ = this->base_angle_ + getSteering(eta);
            this->steering_ = getSteering(eta);
            /*Estimate Gas Input*/
            if(!this->goal_reached_)
            {
                this->velocity = this->reference_v;
            }
        }
    }

    if(goal_reached_)
    {
        velocity = 0.0;
        steering_ = 0.0;

	    twist_cmd_.angular.z = 0.0;
	    twist_cmd_.linear.x  = 0.0;
    }
    this->ackermann_cmd_.drive.steering_angle = this->steering_;
    this->ackermann_cmd_.drive.speed = this->velocity;
    this->ackermann_pub_.publish(this->ackermann_cmd_);
//将得到的角度和线速度转化为差速四轮底盘的线速度和角速度
	//this->twist_cmd_.angular.z = this->velocity * tan(this->steering_) * 2.04;
     this->twist_cmd_.angular.z = (this->velocity * 2 * tan(getSteering(eta)) ) / Lfw;
       // this->twist_cmd_.angular.z = this->velocity * tan(this->steering_) * 2.04;
    //this->twist_cmd_.angular.z = this->steering_ * this->velocity *2.04 ; // 1/L 
    this->twist_cmd_.linear.x = this->velocity;
    this->twist_pub_.publish(this->twist_cmd_);

}


int main(int argc, char **argv) {
    // 创建ROS节点
    ros::init(argc, argv, "PurePursuit");
    // 调用类
    PurePursuit ppc;
    // 多线程工作
    ros::AsyncSpinner spinner(3);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}