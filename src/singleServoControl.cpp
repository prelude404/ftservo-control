#include <ftservoControl/FEETECHservo.h>
#include <ftservoControl/math_tools.h>
#include <math.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_datatypes.h>

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PointStamped.h>


double x, y, z;
using namespace std;



bool flag = false;
int id_up;
int id_down;
double up_status = 225.0;
double down_status = 180.0;
double down_change = 0;
double up_change = 0;
string source_frame;
string target_frame;
string cam;
string serial_str;

tf::StampedTransform base_to_servogroup;
tf::Transform servogroup_to_camera;

Eigen::Matrix4Xd T01 = Eigen::Matrix4d::Identity();
Eigen::Matrix4Xd T12 = Eigen::Matrix4d::Identity();
Eigen::Matrix4Xd T23 = Eigen::Matrix4d::Identity();
Eigen::Matrix4Xd T34 = Eigen::Matrix4d::Identity();
Eigen::Matrix4Xd T_servogroup_to_cam = Eigen::Matrix4d::Identity();
Eigen::Matrix4Xd T_cam_to_coopestimation = Eigen::Matrix4d::Identity();
Eigen::Matrix4Xd T_cam_to_estimation = Eigen::Matrix4d::Identity();

ros::Publisher pub_servogroup_to_cam;
ros::Subscriber sub_cam_to_estimation;
ros::Subscriber sub_cam_to_coopestimation;
tf::StampedTransform servogroup_to_cam;
tf::StampedTransform cam_to_estimation;
tf::StampedTransform cam_to_coopestimation;
geometry_msgs::TransformStamped msg_servogroup_to_cam;

void T_servogroup_to_camera(double id_down, double id_up){
	T01 << cos(id_down), sin(id_down), 0, 0, -sin(id_down), cos(id_down), 0, 0, 0, 0, 1, 38.3/1000, 0, 0, 0, 1;
	T12 << 1, 0, 0, 0, 0, 0, 1, 21.3 / 1000, 0, -1, 0, 39.6 / 1000, 0, 0, 0, 1;
	T23 << cos(id_up), sin(id_up), 0, 0, -sin(id_up), cos(id_up), 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
	T34 << 1, 0, 0, 9.36 / 1000, 0, 0, -1, -59 / 1000, 0, 1, 0, 26 / 1000, 0, 0, 0, 1;
	Eigen::Matrix4d T_correct = Eigen::Matrix4d::Identity();
	T_servogroup_to_cam = T01 * T12 * T23 * T34;
	
}

void T_cam_to_estimation_callback(const geometry_msgs::TransformStamped &msg){
	cam_to_estimation.setOrigin(tf::Vector3(msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z));
	cam_to_estimation.setRotation(tf::Quaternion(msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w));
	flag = true;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "singleServoControl");
	ros::NodeHandle nh;
	
	tf::TransformBroadcaster br;
	tf::TransformListener lr;

	nh.param<int>("singleServoControl/id_up", id_up, 2);
    nh.param<int>("singleServoControl/id_down", id_down, 1);
	nh.param<string>("singleServoControl/target_frame", target_frame, "EstimationfromcamA");
    nh.param<string>("singleServoControl/source_frame", source_frame, "camA");
	nh.param<string>("singleServoControl/cam", cam, "camA");
	nh.param<string>("singleServoControl/serial", serial_str, "/dev/ttyUSB0");
	const char *serial_ = serial_str.c_str();

	// ftServo _servo;


	ros::Rate rate(30);

	// _servo.init(serial_, 2, nh,{id_down, id_up});
	// _servo.move(down_status,id_down);
	// cout<<"down_status:"<<down_status<<endl;
	// _servo.move(up_status,id_up);
	// cout<<"up_status:"<<up_status<<endl;
	// ros::Duration(1).sleep();

	std::cout<<"system has been initialed!"<<std::endl;
	pub_servogroup_to_cam = nh.advertise<geometry_msgs::TransformStamped>("/T_servogroup"+to_string(id_down)+to_string(id_up)+"_to_"+cam, 1);
	// sub_cam_to_estimation = nh.subscribe("/"+cam+"/single_cam_process_ros/ir_mono/T_cam_to_estimation", 1, T_cam_to_estimation_callback);
	
	while (ros::ok())
	{
		//计算电机组到相机的变换
		T_servogroup_to_camera((down_status - 180) / 180 * pi, (up_status - 180) / 180 * pi);

		cout<<"T_servogroup_to_cam"<<endl<<T_servogroup_to_cam<<endl;
		servogroup_to_cam.setOrigin(EigenVector3dToTFVector3(T_servogroup_to_cam.block<3,1>(0,3)));
		servogroup_to_cam.setRotation(EigenQuaterniondToTFQuaternion(Eigen::Quaterniond(T_servogroup_to_cam.block<3,3>(0,0))));
		servogroup_to_cam.stamp_ = ros::Time::now();
		servogroup_to_cam.frame_id_ = "servogroup"+to_string(id_down)+to_string(id_up);
		servogroup_to_cam.child_frame_id_ = cam;
		tf::transformStampedTFToMsg(servogroup_to_cam, msg_servogroup_to_cam);
        pub_servogroup_to_cam.publish(msg_servogroup_to_cam);
        br.sendTransform(msg_servogroup_to_cam);

		// if(flag == false)
		// 	try{
		// 		// 等待变换
		// 		lr.waitForTransform(target_frame, "CoopEstimation", ros::Time(0), ros::Duration(1.0));
		// 		// 查询坐标系关系
		// 		lr.lookupTransform(target_frame, "CoopEstimation", ros::Time(0), cam_to_estimation);

		// 		flag = true;

		// 	}
		// 	catch(tf::TransformException &ex)
		// 	{
		// 		ROS_ERROR("%s", ex.what());
		// 	}


		// if(flag){
			// x = cam_to_estimation.getOrigin().x();
			// y = cam_to_estimation.getOrigin().y();
			// z = cam_to_estimation.getOrigin().z();
			// std::cout <<"(TF_cam_to_estimation)  x:"<< x <<"y:"<< y <<"z:"<< z << std::endl;
			
			//下方舵机旋转
			// if(y <= 0)
			// {
			// 	down_change = abs( atan(y / x) / pi * 180) ;//左负右正
			// }else if(y > 0)
			// {
			// 	down_change = -abs( atan(y / x) / pi * 180) ;//左负右正
			// }
			// down_status = down_change + down_status;
			// // down_status = (abs(down_change) > 2) * down_change + down_status;
			// down_status = min(max(down_status, 50.0), 310.0);
			// _servo.move(down_status,id_down);
			// std::cout<<"down moving angle:"<<down_change<<std::endl;
			// std::cout<<"down angle:"<<down_status<<std::endl;

			// //上方舵机旋转
			// double up_change = atan(z / sqrt(x * x + y * y)) / pi * 180 / 2;//上正下负
			// up_status = up_change + up_status;
			// // up_status = (abs(up_change) > 3) * up_change + up_status;
			// up_status = min(max(up_status, 135.0), 225.0);
			// _servo.move(up_status,id_up);	
			// std::cout<<"up moving angle:"<<up_change<<std::endl;
			// std::cout<<"up angle:"<<up_status<<std::endl;
			// flag = false;
		// }
		// ros::Duration(1).sleep();
		ros::spinOnce();
		rate.sleep();

		
	}
	
	return 0;
}