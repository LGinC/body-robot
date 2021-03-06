﻿
namespace body_robot
{
    class Constants
    {
        /// <summary>
        /// 舵机PWM异常值
        /// </summary>
        public const int INVALID_JOINT_VALUE = -1;

        /// <summary>
        /// PWM最大值
        /// </summary>
        public const int PWM_MAX = 250;

        /// <summary>
        /// position数组长度
        /// </summary>
        public const int POSITION_LENTH = 18;

        /// <summary>
        /// 左大腿向前阀值，小于阀值则做动作
        /// </summary>
        public const int ThighLeft_threshold = 110;//  <


        /// <summary>
        /// 右大腿向前阀值， 大于阀值则做动作
        /// </summary>
        public const int ThighRight_threshold = 150;// >


        /// <summary>
        /// 左髋向左阀值， 大于阀值则做动作
        /// </summary>
        public const int HipLeft_threshold = 150;//    >

        /// <summary>
        /// 右髋向右阀值，小于阀值则做动作
        /// </summary>
        public const int HipRight_threshold = 105;//   <
                                    
        /// <summary>
        /// 发送每一帧由frame_count帧数据过滤而得
        /// </summary>
        public const int frame_count = 4;

        /// <summary>
        /// socket buffer大小
        /// </summary>
        public const int buffer_size = 2048;

        /// <summary>
        /// socket连接超时时间，单位是ms
        /// </summary>
        public const int connect_timeout = 10000;

        /// <summary>
        /// socket接收超时时间，单位ms
        /// </summary>
        public const int receive_timeout = 1000;

        /// <summary>
        /// socket发送超时时间，单位是ms
        /// </summary>
        public const int send_timeout = 1000;

        /// <summary>
        /// 连接状态——成功
        /// </summary>
        public const string connect_success = "已连接";

        /// <summary>
        /// 右走一步阀值
        /// </summary>
        public const int right_shift_threshold = 100;//<

        /// <summary>
        /// 右转阀值
        /// </summary>
        public const int right_turn_threshold = 60;

        /// <summary>
        /// 左走一步阀值
        /// </summary>
        public const int left_shift_threshold = 165;//>

        /// <summary>
        /// 左转阀值
        /// </summary>
        public const int left_turn_threshold = 200;



    }
}
