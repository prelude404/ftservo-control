/*
 * @Author: 黄先 1215399660@qq.com
 * @Date: 2024-02-28 09:36:03
 * @LastEditors: 黄先 1215399660@qq.com
 * @LastEditTime: 2024-02-28 21:02:17
 * @FilePath: /test_ws/src/ftservoControl/src/WritePos.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include <ftservoControl/FEETECHservo.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ftServoWritePos");
	ros::NodeHandle nh;
	int modified_id;
	const char *_serial;
	std::string serial_string;
	nh.param("ftservo/id", modified_id, 1);
	nh.param("ftservo/serial", serial_string, std::string("/dev/ttyUSB0"));
	_serial = serial_string.c_str();
	ftServo _servo;
	// std::vector<int> IDs = {2};
	_servo.init(_serial, 1, nh);/*只连接一个舵机，且其编号是1*/
	//注意！！！初始化会将所有的舵机重置为180deg！！！！
	// _servo.init(_serial, 1, nh, IDs);/*只连接一个舵机，且其编号是2*/
	// _servo.init(_serial, 2, nh);/*连接两个舵机，且其编号是{1,2}*/

	//转动 注意只接受0～360deg
	// _servo.move(30,1);/*1号舵机转动至30deg*/
	// _servo.move(150,2);/*2号舵机转动至150deg*/
	// std::cout<<"Move to 90"<<std::endl;
	// ros::Duration(1.0).sleep();

	//读取id是1的舵机角度
	// std::cout<<"Read 1 "<<_servo.read(1)<<std::endl;
	// ros::Duration(1.0).sleep();

	// 将id是1的舵机重编号为2
	// _servo.rename(1,2);
	// std::cout<<"rename 1 to 2"<<std::endl;
	// ros::Duration(1.0).sleep();
	ROS_INFO("Modification completed. The modified ID is: %d", modified_id);

	ros::spin();
	return 0;
}