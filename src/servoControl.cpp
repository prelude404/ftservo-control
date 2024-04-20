/*
舵机出厂速度单位是0.0146rpm，速度改为V=2400
*/
//位置描述为180度2048，90度1024，实现360度绝对角度控制
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include "SCServo.h"
uint8_t ID[] = {1, 2};
uint8_t rxPacket[4];
int16_t Position[2]={0,0};
int16_t Speed[2]={0,0};
SMS_STS sm_st;
int Constrain_speed = 1100;
int angle_trans(double realangle){
	int res=realangle*2048/180;
	return res;
}
void updatePosVel(){
	sm_st.syncReadPacketTx(ID, sizeof(ID), SMS_STS_PRESENT_POSITION_L, sizeof(rxPacket));//同步读指令包发送
		for(uint8_t i=0; i<sizeof(ID); i++){
			//接收ID[i]同步读返回包
			if(!sm_st.syncReadPacketRx(ID[i], rxPacket)){
				std::cout<<"ID:"<<(int)ID[i]<<" sync read error!"<<std::endl;
				continue;//接收解码失败
			}
			Position[i] = sm_st.syncReadRxPacketToWrod(15);//解码两个字节 bit15为方向位,参数=0表示无方向位
			Speed[i] = sm_st.syncReadRxPacketToWrod(15);//解码两个字节 bit15为方向位,参数=0表示无方向位
			std::cout<<"ID:"<<int(ID[i])<<" Position:"<<Position<<" Speed:"<<Speed<<std::endl;
		}
}
void servo1Callback(std_msgs::Float64 msg){
	double angle=msg.data;
	if(abs(angle)>=1e-3){
		if(angle>180){
			angle = angle-360;
		}
		int step=-angle_trans(angle);
	// std::cout<<step<<std::endl;

		sm_st.WritePosEx(1, step+2048, Constrain_speed, 50);
	}
	
}
void servo2Callback(std_msgs::Float64 msg){
	double angle=msg.data;
	if(abs(angle)>=1e-3){
		int step=angle_trans(angle);
	// std::cout<<step<<std::endl;
	//std::cout<<"Pos2 step error="<<step-Position[1]<<std::endl;
		sm_st.WritePosEx(2, step, Constrain_speed, 50);
	}
	
}
void servo3Callback(std_msgs::Float64 msg){
	double angle=msg.data;
	if(abs(angle)>=1e-3){
		int step=-angle_trans(angle);
	//std::cout<<step<<std::endl;
	//std::cout<<"Pos1 step error="<<step-Position[0]<<std::endl;
		sm_st.WritePosEx(3, 4096+step, Constrain_speed, 50);
	}
	
}
void servo4Callback(std_msgs::Float64 msg){
	double angle=msg.data;
	if(abs(angle)>=1e-3){
		int step=angle_trans(angle);
	//std::cout<<step<<std::endl;
	//std::cout<<"Pos1 step error="<<step-Position[0]<<std::endl;
		sm_st.WritePosEx(4, step, Constrain_speed, 50);
	}
	
}

int main(int argc, char **argv)
{
	if(argc<2){
        std::cout<<"argc error!"<<std::endl;
        return 0;
	}
	ros::init(argc,argv,"ftServo");
	ros::NodeHandle nh;
	ros::Subscriber servo1_sub=nh.subscribe<std_msgs::Float64>("/servo1",1,servo1Callback);
	ros::Subscriber servo2_sub=nh.subscribe<std_msgs::Float64>("/servo2",1,servo2Callback);
	ros::Subscriber servo3_sub=nh.subscribe<std_msgs::Float64>("/servo3",1,servo3Callback);
	ros::Subscriber servo4_sub=nh.subscribe<std_msgs::Float64>("/servo4",1,servo4Callback);
	std::cout<<"serial:"<<argv[1]<<std::endl;
    if(!sm_st.begin(1000000, argv[1])){
        std::cout<<"Failed to init sms/sts motor!"<<std::endl;
        return 0;
    }
	sm_st.WritePosEx(1, 2048, Constrain_speed, 50);
	sm_st.WritePosEx(2, 0, Constrain_speed, 50);
	sm_st.WritePosEx(3, 4096, Constrain_speed, 50);
	sm_st.WritePosEx(4, 0, Constrain_speed, 50);
	//sm_st.syncReadBegin(sizeof(ID), sizeof(rxPacket));
	//usleep(1000*20);
	ros::Rate looprate(125);
	while(ros::ok()){
		//updatePosVel();
		ros::spinOnce();
		looprate.sleep();
	}
	sm_st.end();
	return 1;
}

