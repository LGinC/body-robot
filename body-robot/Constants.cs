
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
        /// 左大腿蹲下阀值，小于阀值蹲下
        /// </summary>
        public const int ThgihtLeft_down = 65;// <

        /// <summary>
        /// 右大腿向前阀值， 大于阀值则做动作
        /// </summary>
        public const int ThighRight_threshold = 150;// >

        /// <summary>
        /// 右大腿蹲下阀值，大于阀值蹲下
        /// </summary>
        public const int ThighRight_dwon = 185;// >

        /// <summary>
        /// 左髋向左阀值， 大于阀值则做动作
        /// </summary>
        public const int HipLeft_threshold = 150;//    >

        /// <summary>
        /// 右髋向右阀值，小于阀值则做动作
        /// </summary>
        public const int HipRight_threshold = 105;//   <

        public const string init_PWM = "sb115006122133244128229023130000114000231153028103000";
    }
}
