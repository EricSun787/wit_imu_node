#include "../include/wit_imu_9dof/wit_imu.h"
#include <iostream>

serial::Serial sp;
bool isconnect(false);
bool data_update(const unsigned char * incoming, unsigned int numberOfIncoming)
{
    if(!(numberOfIncoming > 0))
    {
        findHeader=false;
        state = waitingForHead;
        return false;
    }
    bool found_packet(false);
    static unsigned char cs(0);
    uint8_t subpay_size=0;
    uint8_t checksum=0;

    switch(state)
    {
        case(waitingForHead):
             if(findHeader==false && (incoming[0] == dataheader))
                {
                    findHeader = true;
                    ready_get = 10;
                    state = waitingForPayload;
                }
        break;

        case(waitingForPayload):
        {
            switch(incoming[0])
            {
                case IDHeader::angle:
                    cs = 0x55;
                    for(int i=0;i<9;i++)
                    {
                        cs+=incoming[i];
                    }
                    checksum = incoming[9];
                    if(cs == checksum)
                    {
                        memcpy(&stcAngle,&incoming[1],8);

                        //printf("%.3f\r\n", (float)stcAngle.Angle[2]/32768*180);
                        //RPY[0] = (float)stcAngle.Angle[0]/32768*180;
                        //RPY[1] = (float)stcAngle.Angle[1]/32768*180;
                        //RPY[2] = (float)stcAngle.Angle[2]/32768*180;
                        RPY[0] = (float)stcAngle.Angle[0]/32768*PI;
                        RPY[1] = (float)stcAngle.Angle[1]/32768*PI;
                        RPY[2] = (float)stcAngle.Angle[2]/32768*PI;

                      //printf("%.3f  %.3f  %.3f  \r\n", RPY[0], RPY[1], RPY[2]);

                        findHeader = false;
                        state = waitingForHead;
                        isconnect = true;
                        found_packet = true;
                        ready_get = 1;
                    }
                    else
                    {
                       ROS_WARN("Angle CheckSum ERROR! cs:%d  data:%d",cs,checksum);
                       return false;
                    }
                break;
                case IDHeader::angle_acc:
                    cs = 0x55;
                    for(int i=0;i<9;i++)
                    {
                        cs+=incoming[i];
                    }
                    checksum = incoming[9];
                    if(cs == checksum)
                    {
                        memcpy(&stcAcc,&incoming[1],8);

                        
                        findHeader = false;
                        state = waitingForHead;
                        isconnect = true;
                        found_packet = true;
                        ready_get = 1;
                    }
                    else
                    {
                       ROS_WARN("Angle Acc CheckSum ERROR! cs:%d  data:%d",cs,checksum);
                       return false;
                    }
                break;
                case IDHeader::angle_speed:
                    cs = 0x55;
                    for(int i=0;i<9;i++)
                    {
                        cs+=incoming[i];
                    }
                    checksum = incoming[9];
                    if(cs == checksum)
                    {
                        memcpy(&stcGyro,&incoming[1],8);
                        
                        findHeader = false;
                        state = waitingForHead;
                        isconnect = true;
                        found_packet = true;
                        ready_get = 1;
                    }
                    else
                    {
                       ROS_WARN("Angle Speed CheckSum ERROR! cs:%d  data:%d",cs,checksum);
                        return false;
                    }
                break;
                default:
                        ROS_WARN("Can not find DATA_ID: %d",incoming[0]);
                break;
            }
                    

        }
    }

    return found_packet;

}

int main(int argc, char** argv)
{
    
    ros::init(argc, argv, "wit_imu"); 
    ros::NodeHandle nh;
    ros::NodeHandle nh_p("~");
    sensor_msgs::Imu imu_pub_data;

    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("/imu",10);

    std::string port = "/dev/ttyUSB0";
    ros::param::get("~serial_port", port);

    try
    {
       //创建timeout
       serial::Timeout to = serial::Timeout::simpleTimeout(1000);
       //设置要打开的串口名称
       sp.setPort(port);
       //设置串口通信的波特率
       sp.setBaudrate(115200);
       //串口设置timeout
       sp.setTimeout(to);
       //打开串口
       sp.open();
    }
    catch(serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port.");
        return -1;
    }


    if(sp.isOpen())
    {
        ROS_INFO_STREAM("Device is opened.");
    }
    else
    {
        return -1;
    }
    
    
    ros::Rate loop_rate(200);
    ros::Time last_getdata = ros::Time::now();
    uint8_t temp_buffer[256];
    uint8_t buffer[256];
    uint8_t i =0;

    while(ros::ok())
    {
        size_t n = sp.read(temp_buffer, ready_get);
        if(n==0)
        {
            std::cout<<"waiting for imu data 10s"<<std::endl;
            if((ros::Time::now() - last_getdata) > ros::Duration(10.0))
            {
                std::cout<<"imu data time out!close serial!"<<std::endl;
                sp.close();
                ros::shutdown();
            } continue;
        }
        if(data_update(temp_buffer,n))
        {
            ROS_INFO_ONCE("IMU Started!");
            /********航向初始化部分********/

            /****************************/
            last_getdata = ros::Time::now();
            
            imu_pub_data.header.frame_id = "/imu";
            imu_pub_data.header.stamp = ros::Time::now();


            imu_pub_data.angular_velocity.x = (float)stcGyro.w[0]/32768*2000*(PI/180);
            imu_pub_data.angular_velocity.y = (float)stcGyro.w[1]/32768*2000*(PI/180);
            imu_pub_data.angular_velocity.z = (float)stcGyro.w[2]/32768*2000*(PI/180);

            imu_pub_data.linear_acceleration.x = (float)stcAcc.a[0]/32768*16*9.8;
            imu_pub_data.linear_acceleration.y = (float)stcAcc.a[1]/32768*16*9.8;
            imu_pub_data.linear_acceleration.z = (float)stcAcc.a[2]/32768*16*9.8;

            imu_pub_data.orientation = tf::createQuaternionMsgFromRollPitchYaw(RPY[0],RPY[1],RPY[2]);

            imu_pub.publish(imu_pub_data);
        }
    }


}
