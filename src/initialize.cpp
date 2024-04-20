/*
 * @Author: 黄先 1215399660@qq.com
 * @Date: 2024-02-28 09:36:03
 * @LastEditors: 黄先 1215399660@qq.com
 * @LastEditTime: 2024-02-28 20:48:17
 * @FilePath: /test_ws/src/ftservoControl/src/initialize.cpp
 * @Description: initialization of modified_id-th servo
 */
#include <ftservoControl/FEETECHservo.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "initialize");
	ros::NodeHandle nh;
	int modified_id ;
	const char *_serial;
	std::string serial_string;
	nh.param("/initialize/ftservo/id", modified_id, 1);
	nh.param("/initialize/ftservo/serial", serial_string, std::string("/dev/ttyUSB0"));
	std::cout<<modified_id<<std::endl;
	_serial = serial_string.c_str();
	ftServo _servo;
	std::vector<int> ID_list = {-1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
	_servo.init(_serial, 17, nh, ID_list);
	_servo.ping(-1);
	_servo.ping(0);
	_servo.ping(1);
	_servo.ping(2);
	_servo.ping(3);
	_servo.ping(4);
	_servo.ping(5);
	_servo.ping(6);
	_servo.ping(7);
	_servo.ping(8);
	_servo.ping(9);
	_servo.ping(10);
	_servo.ping(11);
	_servo.ping(12);
	_servo.ping(13);
	_servo.ping(14);
	_servo.ping(15);
	
	_servo.rename(1, modified_id);
	_servo.reset(modified_id);
	_servo.ping(1);
	ROS_INFO("Modification completed. The modified ID is: %d", modified_id);
	return 0;
}