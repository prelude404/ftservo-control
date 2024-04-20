/*
 * @Author: hx2020hx 1215399660@qq.com
 * @Date: 2024-02-28 09:40:07
 * @LastEditors: 黄先 1215399660@qq.com
 * @LastEditTime: 2024-02-28 20:38:34
 * @FilePath: /test_ws/src/ftservoControl/src/FEETECHservo.cpp
 * @Description: API for FEETECHServo contol
 */
#include <ftservoControl/FEETECHservo.h>
void ftServo::init(const char *serial, int num_servos, ros::NodeHandle &nh)
{
    nh_ = nh;
    num = num_servos;
    for (int i = 1; i <= num_servos; i++)
    {
        id_list.push_back(i);
    }
    if (!sm_st.begin(100000, serial))
    {
        std::cout << "Failed to init sms/sts motor!" << std::endl;
        return;
    }
    current_angle.resize(num);
    target_angle.resize(num);
    current_speed.resize(num);
    std::fill(target_angle.begin(), target_angle.end(), 180.0);
    std::fill(current_speed.begin(), current_speed.end(), 2400);
    // update_timer = nh_.createTimer(ros::Duration(0.025), &ftServo::update_servo_state, this);
}

void ftServo::init(const char *serial, int num_servos, ros::NodeHandle &nh, std::vector<int> ID_list)
{
    nh_ = nh;
    num = num_servos;
    id_list = ID_list;
    if (!sm_st.begin(100000, serial))
    {
        std::cout << "Failed to init sms/sts motor!" << std::endl;
        return;
    }
    current_angle.resize(num);
    target_angle.resize(num);
    current_speed.resize(num);
    std::fill(target_angle.begin(), target_angle.end(), 180.0);
    std::fill(current_speed.begin(), current_speed.end(), 2400);
    // update_timer = nh_.createTimer(ros::Duration(0.025), &ftServo::update_servo_state, this);
}

void ftServo::move(double angle, int id, double speed)
{
    int index;
    bool sucess = find_id(id, index);
    if (!sucess)
    {
        ROS_ERROR("Wrong input of id");
        return;
    }
    target_angle[index] = angle;
    current_speed[index] = speed;

    int step = angleTrans(target_angle[index]);
    sm_st.WritePosEx(id_list[index], step, current_speed[index], 50);
}

double ftServo::read(int id)
{
    int index;
    bool sucess = find_id(id, index);
    if (!sucess)
    {
        ROS_ERROR("Wrong input of id");
        return -1;
    }
    int step = sm_st.ReadPos(id);
    double Pos = -1;
    if (step != -1)
    {
        Pos = stepTrans(step);
    }
    else
    {
        ROS_ERROR("read pos err");
    }
    return Pos;
}

void ftServo::reset(int id)
{
    int index;
    bool sucess = find_id(id, index);
    if (!sucess)
    {
        ROS_ERROR("Wrong input of id");
        return;
    }
    sm_st.CalibrationOfs(id);
    ROS_INFO("Reset to 180 deg");
}

void ftServo::rename(int id, int modified_id)
{
    int index;
    bool sucess = find_id(id, index);
    if (!sucess)
    {
        ROS_ERROR("Wrong input of id");
        return;
    }

    id_list[index] = modified_id;
    sm_st.unLockEprom(id);
    sm_st.writeByte(id, SMSBL_ID, modified_id);
    sm_st.LockEprom(modified_id);
}

void ftServo::update_servo_state(const ros::TimerEvent &)
{

    for (int i = 0; i < num; i++)
    {
        current_angle[i] = read(id_list[i]);
        // std::cout<<current_angle[i]<<std::endl;
        // std::cout<<target_angle[i]<<std::endl;
        if (fabs(current_angle[i] - target_angle[i]) > 0.1)
        {
            int step = angleTrans(target_angle[i]);
            sm_st.WritePosEx(id_list[i], step, current_speed[i], 50);
        }
    }
}
bool ftServo::find_id(int id, int &index)
{
    for (size_t i = 0; i < id_list.size(); ++i)
    {
        if (id_list[i] == id)
        {
            index = i;
            return true;
        }
    }
    return false;
}

bool ftServo::ping(int id)
{
    int index;
    bool sucess = find_id(id, index);
    if (!sucess)
    {
        ROS_ERROR("Wrong input of id");
        return false;
    }
    int ID = sm_st.Ping(id);
    if (ID != -1)
    {
        ROS_INFO("ID:%i", ID);
        return true;
    }
    else
    {
        ROS_WARN("Ping ID wrong");
        return false;
    }
}