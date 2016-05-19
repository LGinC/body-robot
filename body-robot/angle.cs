using System;
using Microsoft.Kinect;

namespace body_robot
{
    class angle
    {
        /// <summary>
        /// 关节过滤阀值缓冲池,大小为body所有关节数
        /// </summary>
        public Joint[] joint_filter = new Joint[Body.JointCount];

        /// <summary>
        /// 缓冲池是否初始化标志
        /// </summary>
        private bool IsFilterInit = false; 

        /// <summary>
        /// 存放knee的角度，用于计算anklePWM
        /// </summary>
        private double acos_knee = 0;

        /// <summary>
        /// 对所有关节结构体进行限幅滤波
        /// </summary>
        /// <param name="j">关节节点结构体</param>
        /// <returns>返回过滤后的节点结构体</returns>
        public Joint filter(Joint j )
        {
            int type = (int)j.JointType;
            if(this.IsFilterInit == false)  //如果阀值未初始化，则将传入的节点作为阀值
            {
                this.joint_filter[type] = j;
            }
            else
            {
                if(j.TrackingState >= this.joint_filter[type].TrackingState) //传入的节点可信度大于阀值，则更新阀值，否则则将传入的节点赋值为阀值节点
                {                                                           //当传入节点坐标与原阀值节点坐标差大于0.05则更新阀值
                    double re_x, re_y, re_z, x, y, z;
                    re_x = joint_filter[type].Position.X;
                    re_y = joint_filter[type].Position.Y;
                    re_z = joint_filter[type].Position.Z;
                    x = j.Position.X;
                    y = j.Position.Y;
                    z = j.Position.Z;
                    double diff_x = x - re_x;
                    double diff_y = y - re_y;
                    double diff_z = z - re_z;
                    if ((diff_x < 0.05 && diff_x > -0.05) || (diff_y < 0.05 && diff_y > -0.05) || (diff_y < 0.05 && diff_y > -0.05))
                    {
                        j = this.joint_filter[type];
                    }
                    else
                    {
                        this.joint_filter[type] = j;
                    }

                }
                else
                {
                    this.joint_filter[type] = j;
                }
            }
            return j;    
        }

        ///// <summary>
        ///// 
        ///// </summary>
        ///// <param name="x"></param>
        ///// <param name="y"></param>
        ///// <param name="re_x"></param>
        ///// <param name="re_y"></param>
        ///// <returns></returns>
        //        public static int ToAngle(double x, double y, double re_x, double re_y)
        //        {          
        //            double a = Math.Sqrt(Math.Pow(re_x + 1, 2));
        //            double c = Math.Sqrt(Math.Pow(x + 1, 2) + Math.Pow(y - re_y, 2));
        //            double b = Math.Sqrt(Math.Pow(x - re_x, 2) + Math.Pow(y - re_y, 2));
        //            double cos = (Math.Pow(a, 2) + Math.Pow(b, 2)) / (2 * a * b);
        //            //          cos=sqrt((x-re_x)^2)      /    sqrt((x - re_x)^2) +   sqrt(y - re_y)^2) )
        //            double ret = cos + 1;
        //            return (int)(ret / 2 * Constants.PWM_MAX);
        //        }

        //public static int ToAngle(double x, double y, double z, double re_x, double re_y, double re_z)
        //{
        //    //A(re_x, re_y, re_z)   B(x, y, z) C(re_x, 0 , re_z)
        //    double c = Math.Sqrt(Math.Pow(x - re_x, 2) + Math.Pow(y - re_y, 2) + Math.Pow(z - re_z, 2));
        //    //Sqrt((x - re_x)^2 + (y - re_y)^2 + (z - re_z)^2)
        //    double a = Math.Sqrt(Math.Pow(x - re_x, 2) + Math.Pow(y, 2) + Math.Pow(z - re_z, 2));
        //    //Sqrt((x - re_x)^2 + (y - 0)^2 + (z - re_z)^2)
        //    double b = Math.Sqrt(Math.Pow(re_y, 2));
        //    //Sqrt(re_y^2)
        //    double cos = (Math.Pow(a, 2) + Math.Pow(c, 2) - Math.Pow(a, 2)) / (2 * b * c);
        //    //cos = (a ^ 2 + b ^ 2 + c ^ 2) / 2ab;
        //    if (a + b <c || a + c < b || b + c < a)
        //    {
        //        Console.WriteLine("a:" + a + "\tb:" + b + "\tc:" + c + "can not build a triangle");
        //    }
        //  Console.WriteLine("a:" + a + "\tb:" + b + "\tc:" + c);
        //  Console.WriteLine("cos:"+cos+"\t");
        //    double ret = cos + 1;
        //    return (int)(ret /2 * Constants.PWM_MAX);
        //}
        
        /// <summary>
        /// 计算肩部部舵机所需的PWM信号
        /// </summary>
        /// <param name="s">shoulder 肩部关节结构体</param>
        /// <param name="e">elbow 肘部关节结构体</param>
        /// <param name="h">hand 手部关节结构体</param>
        /// <returns>PWM信号 0-Constants.PWM_MAX</returns>

        public int ToPWMshoulder( Joint s, Joint e, Joint h )//此处传入关节结构体，可获取关节类型来确定是左手还是右手
        {           
            CameraSpacePoint arm,       hand,   n1,                  n2;
            //               胳膊向量   手向量  前二者形成面的法向量   Y轴负方向向量
            double norm_n1,    norm_n2,          cos = 0,         acos,    r;
            //      法向量模长  Y轴方向向量模长    法向量Y轴夹角余弦 反余弦   模长积
            int PWM = 0;
            arm.X = s.Position.X - e.Position.X;//arm = s - e,方向是e->s
            arm.Y = s.Position.Y - e.Position.Y;
            arm.Z = s.Position.Z - e.Position.Z;
            hand.X = h.Position.X - e.Position.X;//hand = h - e,方向是e->h
            hand.Y = h.Position.Y - e.Position.Y;
            hand.Z = h.Position.Z - e.Position.Z;
            n1.X = arm.Y * hand.Z - arm.Z * hand.Y;//arm和hand形成面的法向量
            n1.Y = arm.Z * hand.X - arm.X * hand.Z;
            n1.Z = arm.X * hand.Y - arm.Y * hand.X;
            n2.X = 0;//Y轴负方向
            n2.Y = -100;
            n2.Z = 0;
            norm_n1 = Math.Sqrt(n1.X * n1.X + n1.Y * n1.Y + n1.Z * n1.Z);//法向量模长
            norm_n2 = Math.Sqrt(n2.X * n2.X + n2.Y * n2.Y + n2.Z * n2.Z);//Y轴负方向向量模长
            r = norm_n1* norm_n2;//模长积
            if (r > 0)//积大于0，表示计算有效
            {
                cos = ( n1.X * n2.X + n1.Y * n2.Y + n1.Z * n2.Z ) / r;//计算二者夹角余弦
                //Console.WriteLine("{0} cos:{1}\t", s.JointType.ToString(), cos);
                if (cos <= 1 && cos >= -1)//余弦在有效值内
                {
                    acos = Math.Acos(cos);//计算反余弦，即夹角，0-Pi
                    if (s.JointType == JointType.ShoulderRight)//如果是右肩
                    {
                        if (cos >= 0)//夹角小于90°
                        {
                            if (h.Position.Y > s.Position.Y)
                                acos += Math.PI / 2;
                            else
                                acos = Math.PI / 2 - acos;
                            PWM = (int)(acos / Math.PI * Constants.PWM_MAX);
                            
                        }
                    }
                    else//左肩
                    {
                        if (cos <= 0)
                        {
                            if (h.Position.Y > s.Position.Y)
                                acos = Math.PI - acos + Math.PI / 2;//acos = (3/4)pi - acos
                            else
                                acos -= Math.PI / 2;//acos = acos - pi / 2
                            PWM =  Constants.PWM_MAX - (int)(acos / Math.PI * Constants.PWM_MAX);
                        }
                       
                    }
                    return PWM;
                }
                
            }
            Console.WriteLine("{0} invalid cos:{1}\t", s.JointType.ToString(),cos);
            return Constants.INVALID_JOINT_VALUE;//返回异常值
        }
        
        /// <summary>
        /// 计算肘部舵机PWM信号
        /// </summary>
        /// <param name="s">肩部关节坐标结构体</param>
        /// <param name="e">肘部关节坐标结构体</param>
        /// <returns>PWM信号 0-Constants.PWM_MAX</returns>
        public int ToPWM_elbow(Joint s, Joint e)
        {
            CameraSpacePoint n1, n2;
            double norm_n1, norm_n2, cos = 0, acos, r;
            int PWM = 0; 
            n1.X = e.Position.X - s.Position.X;//方向为s->e
            n1.Y = e.Position.Y - s.Position.Y;
            n1.Z = 0;
            n2.X = 0;
            n2.Y = 100;
            n2.Z = 0;
            norm_n1 = Math.Sqrt(n1.X * n1.X + n1.Y * n1.Y + n1.Z * n1.Z);//n1的模长
            norm_n2 = Math.Sqrt(n2.X * n2.X + n2.Y * n2.Y + n2.Z * n2.Z);//n2的模长       
            r = norm_n1 * norm_n2;
            if(r > 0)
            {
                cos = ( n1.X * n2.X + n1.Y * n2.Y + n1.Z * n2.Z ) / r;
                //Console.WriteLine("{0} cos:{1}\t", e.JointType.ToString(), cos);
                if (cos > -1 && cos < 1)
                {
                    acos = Math.Acos(cos);
                    if(cos < 0)//和Y轴正方向夹角小于90°
                    {
                        acos = Math.PI - acos;
                    }                    
                    PWM =  (int)( acos / Math.PI * Constants.PWM_MAX );
                    if (s.JointType == JointType.ShoulderLeft)//左手 是250 - 0
                    {
                        return Constants.PWM_MAX - PWM;
                    }
                    else//右手0 - 250
                    {
                        return PWM;
                    }
                }                
            }
            Console.WriteLine("{0}: invalid cos:{1}\t", e.JointType.ToString(),cos);
            return Constants.INVALID_JOINT_VALUE;
        }

        /// <summary>
        /// 计算手部舵机PWM信号
        /// </summary>
        /// <param name="s">肩部关节坐标结构体</param>
        /// <param name="e">肘部关节坐标结构体</param>
        /// <param name="h">手部关节坐标结构体</param>
        /// <returns>PWM信号 0-Constants.PWM_MAX</returns>
        public int ToPWM_hand( Joint s, Joint e, Joint h )
        {
            CameraSpacePoint n1, n2;
            double norm_n2, norm_n1, cos = 0, acos, r;
            int PWM;
            n1.X = s.Position.X - e.Position.X;
            n1.Y = s.Position.Y - e.Position.Y;
            n1.Z = s.Position.Z - e.Position.Z;
            n2.X = h.Position.X - e.Position.X;
            n2.Y = h.Position.Y - e.Position.Y;
            n2.Z = h.Position.Z - e.Position.Z;
            norm_n1 = Math.Sqrt(n1.X * n1.X + n1.Y * n1.Y + n1.Z * n1.Z);
            norm_n2 = Math.Sqrt(n2.X * n2.X + n2.Y * n2.Y + n2.Z * n2.Z);
            r = norm_n1 * norm_n2;
            if(r > 0)
            {
                cos = ( n1.X * n2.X + n1.Y * n2.Y + n1.Z * n2.Z ) / r;
                //Console.WriteLine("{0} cos:{1}\t", h.JointType.ToString(),cos);
                if (cos >= -1 && cos <= 1)
                {
                    if (cos > 0)//小于90°则等于90°
                        cos = 0;
                    
                    acos = Math.Acos(cos);
                    acos -= Math.PI / 2;
                    PWM =  (int)( acos / (Math.PI / 2) * (Constants.PWM_MAX / 2) );
                    if (s.JointType == JointType.ShoulderLeft)   //如果为左手
                    {
                        return Constants.PWM_MAX - PWM;
                    }
                    else
                    {
                        return PWM;
                    }
                }
               
            }
            Console.WriteLine("{0} invalid cos:{1}\t", h.ToString(),cos);
            return Constants.INVALID_JOINT_VALUE;
        }

        /// <summary>
        ///  计算臀部舵机PWM信号
        /// </summary>
        /// <param name="h">hip臀关节结构体</param>
        /// <param name="k">knee膝关节结构体</param>
        /// <returns>PWM信号 0-Constants.PWM_MAX</returns>
        public int ToPWM_hip(Joint h, Joint k)
        {
            PointF thigh, n1;
            double norm_thigh, norm_n1, cos = 0, acos, r;
            int PWM;
            thigh.X = k.Position.X - h.Position.X;//大腿向量=hip - knee,这里只用到了XoY平面
            thigh.Y = k.Position.Y - h.Position.Y;
            n1.X = 100;//标准向量为x轴
            n1.Y = 0;

            norm_thigh = Math.Sqrt(thigh.X * thigh.X + thigh.Y * thigh.Y);//大腿向量的模长
            norm_n1 = Math.Sqrt(n1.X * n1.X + n1.Y * n1.Y);//x轴向量的模长
            r = norm_n1 * norm_thigh;//模长之积
            if (r > 0.0)//若模长积大于0才进行计算，否则返回异常值
            {
                cos = (thigh.X * n1.X + thigh.Y * n1.Y) / r;//大腿向量与x轴夹角的cos = 大腿向量和x轴向量之积 / 二者模长之积
                //Console.WriteLine("k.x:{0} k.y:{1}      h.x:{2} h.y:{3}", k.Position.X, k.Position.Y, h.Position.X, h.Position.Y);
                //Console.WriteLine(h.JointType.ToString() + " cos:" + cos);
                if (cos < 1 && cos > -1)//余弦值小于1大于-1
                {
                    acos = Math.Acos(cos);//反余弦，得到角度
                    PWM = (int)(acos / Math.PI * Constants.PWM_MAX);//PWM = (acos / Pi) * 250
                }
                else//大于1或者小于-1返回异常值
                {
                    return Constants.INVALID_JOINT_VALUE;
                }
                return PWM;
            }
            Console.WriteLine("{0} invalid cos:{1}\t", h.ToString(),cos);
            return Constants.INVALID_JOINT_VALUE;
        }
        /// <summary>
        ///  计算大腿部舵机PWM信号
        /// </summary>
        /// <param name="h">hip臀关节结构体</param>
        /// <param name="k">knee膝关节结构体</param>
        /// <returns>PWM信号 0-Constants.PWM_MAX</returns>
        public int ToPWM_thigh(Joint h, Joint k)
        {
            CameraSpacePoint thigh, n1;//大腿向量和标准向量
            double norm_thigh, norm_n1, cos = 0, acos, r;
            //    大腿向量模长 标准向量模长   余弦  反余弦  模长积
            int PWM;//PWM值
            thigh.Z = k.Position.Z - h.Position.Z;//大腿向量=hip - knee,这里只用到了ZoY平面
            thigh.Y = k.Position.Y - h.Position.Y;
            n1.Z = 100;//标准向量为z轴正方向
            n1.Y = 0;

            norm_thigh = Math.Sqrt(thigh.Z * thigh.Z + thigh.Y * thigh.Y);//大腿向量的模长
            norm_n1 = Math.Sqrt(n1.Z * n1.Z + n1.Y * n1.Y);//x轴向量的模长
            r = norm_n1 * norm_thigh;//模长之积
            if (r > 0.0)//若模长积大于0才进行计算，否则返回异常值
            {
                cos = (thigh.Z * n1.Z + thigh.Y * n1.Y) / r;//大腿向量与z轴夹角的cos = 大腿向量和z轴向量之积 / 二者模长之积
                //Console.WriteLine("k.y:{0} k.z:{1}      h.y:{2} h.z:{3}", k.Position.Y, k.Position.Z, h.Position.Y, h.Position.Z);
                //Console.WriteLine(k.JointType.ToString() + " cos:" + cos);
                if (cos < 1 && cos > -1)//余弦值小于1大于-1,否则返回异常值
                {
                    acos = Math.Acos(cos);//反余弦，得到角度，范围是0-PI                   
                    PWM = (int)(acos / Math.PI * Constants.PWM_MAX);//PWM = (acos / PI) * 250   
                    //Console.WriteLine(h.JointType.ToString() + " acos:" + acos);
                    if (h.JointType == JointType.HipLeft)//若是左腿
                    {
                        PWM = Constants.PWM_MAX - PWM;//PWM = 250 - PWM，左腿是250 -> 0,右腿是0 -> 250变化
                    }                                    
                    return PWM;
                }               
            }
            Console.WriteLine("thigh invalid cos:{0}\t", cos);
            return Constants.INVALID_JOINT_VALUE;
        }

        /// <summary>
        /// 计算膝关节舵机PWM信号
        /// </summary>
        /// <param name="h">hip臀关节结构体</param>
        /// <param name="k">knee膝关节结构体</param>
        /// <param name="a">ankle踝关节结构体</param>
        /// <returns>PWM信号 0-Constants.PWM_MAX</returns>
        public int ToPWM_knee(Joint h, Joint k, Joint a)
        {
            CameraSpacePoint thigh, calf;//大腿小腿向量
            double norm_thigh, norm_calf,   cos = 0, acos, r;
            //     大腿向量模长 小腿向量模长  余弦 反余弦 模长积
            int PWM;//PWM值
            thigh.X = h.Position.X - k.Position.X;//thigh = hip - knee
            thigh.Y = h.Position.Y - k.Position.Y;
            thigh.Z = h.Position.Z - k.Position.Z;
            calf.X = a.Position.X - k.Position.X;//thigh = ankle - knee
            calf.Y = a.Position.Y - k.Position.Y;
            calf.Z = a.Position.Z - k.Position.Z;

            norm_thigh = Math.Sqrt(thigh.X * thigh.X + thigh.Y * thigh.Y + thigh.Z * thigh.Z);//大腿向量模长
            norm_calf = Math.Sqrt(calf.X * calf.X + calf.Y * calf.Y + calf.Z * calf.Z);//小腿向量模长
            r = norm_thigh * norm_calf;//模长之积

            if(r > 0)//模长积大于0才进行处理，否则返回异常值
            {
                cos = (thigh.X * calf.X + thigh.Y * calf.Y + thigh.Z * calf.Z) / r;//大小腿向量夹角余弦
              //Console.WriteLine("k.x:{0} k.y:{1} k.z{2}      h.x:{3} h.y:{4} h.z:{5}", k.Position.X, k.Position.Y, k,Position.Z, h.Position.X, h.Position.Y, h.Position.Z);
              //Console.WriteLine(k.JointType.ToString() + " cos:" + cos);
                if (cos < 1 && cos > -1)//余弦值小于1大于-1,否则返回异常值
                {
                    acos = Math.Acos(cos);//反余弦，得到角度，范围是0-PI      
                    acos_knee = acos;             
                    PWM = (int)(acos / Math.PI * Constants.PWM_MAX);//PWM = (acos / PI) * 250
                    if(k.JointType == JointType.KneeLeft)//左腿时
                    {
                        PWM = Constants.PWM_MAX - PWM;//PWM为0时，角度为180°，所以用PWM_MAX - PWM
                    }
                    //Console.WriteLine(k.JointType.ToString() + " PWM:" + PWM);
                    return PWM;
                }
            }
            Console.WriteLine("{0} invalid cos:{1}\t", k.ToString(), cos);
            return Constants.INVALID_JOINT_VALUE;
        }

        /// <summary>
        /// 踝部舵机计算，角度等于膝部/2
        /// </summary>
        /// <param name="a">踝部关节结构体</param>
        /// <returns>PWM信号 0-Constants.PWM_MAX</returns>
        public int ToPWM_ankle(Joint a)
        {
            int PWM = 0;
            double acos = acos_knee / 2.0;
            PWM = (int)(acos / Math.PI * Constants.PWM_MAX);
            if(a.JointType == JointType.AnkleRight)//右踝 是250到0
            {
                PWM = Constants.PWM_MAX - PWM;
            }
            return PWM;
        }

        public int ToPWM_foot()
        {
            return Constants.INVALID_JOINT_VALUE;
        }

        /// <summary>
        /// 脚部姿态检测
        /// </summary>
        /// <param name="position">所有舵机的PWM数组</param>
        public void PoseDect(ref int[] position)
        {
           // Console.WriteLine("here is Pose");
            //foreach(var p in position)
            //{
            //    Console.Write(p+ " ");
            //}
            if(Constants.INVALID_JOINT_VALUE != position[ (int)servos.ThighLeft])//若左大腿舵机不为异常值
            {
                if(position[(int)servos.ThighLeft ] <= Constants.ThgihtLeft_down  && position[(int)servos.ThighRight ] >= Constants.ThighRight_dwon)//若左右大腿舵机过阀值，即夹角小于阀值则为蹲
                {
                    position[Constants.POSITION_LENTH - 1] = (int)pose.squat;//蹲
                    return;
                }
                
                
                if(Constants.INVALID_JOINT_VALUE != position[(int)servos.HipLeft ])//若左髋关节舵机不为异常值
                {
                    if(position[(int)servos.HipLeft ] >= Constants.HipLeft_threshold)//若左髋关节大于于阀值，即夹角小于阀值，左腿偏左
                    {
                        if(position[(int)servos.ThighLeft ] <= Constants.ThighLeft_threshold)//若左大腿舵机小于阀值,即左腿偏向前
                        {
                            if(Math.Abs((position[(int)servos.HipLeft ] - Constants.HipLeft_threshold)) > Math.Abs((Constants.ThighLeft_threshold - position[(int)servos.ThighLeft])))//向左分量大于向前分量
                            {
                                position[Constants.POSITION_LENTH - 1] = (int)pose.walk_left;//则左行
                                return;
                            }
                            else//否则
                            {
                                position[Constants.POSITION_LENTH - 1] = (int)pose.walk_front;//前行
                                return;
                            }
                        }
                    }
                    //else//左髋关节大于阀值，即左脚没有向左倾向
                    //{
                    //    if(position[(int)servos.ThighLeft ] <= Constants.ThighLeft_threshold)//若左大腿舵机小于阀值,即左腿偏向前
                    //    {
                    //        position[Constants.POSITION_LENTH - 1] = (int)pose.walk_front;//前行
                    //        return;
                    //    }
                    //}
                }
                //else//若左髋关节为异常值
                //{
                //    if (position[(int)servos.ThighLeft ] <= Constants.ThighLeft_threshold)//若左大腿舵机小于阀值,即左腿偏向前
                //    {
                //        position[Constants.POSITION_LENTH - 1] = (int)pose.walk_front;//前行
                //        return;
                //    }
                //    else
                //    {
                //        position[Constants.POSITION_LENTH - 1] = (int)pose.stand_up;//起立
                //        return;
                //    }
                //}           
            }

            if (Constants.INVALID_JOINT_VALUE != position[(int)servos.ThighRight])//若右大腿舵机不为异常值
            {
                if (Constants.INVALID_JOINT_VALUE != position[(int)servos.HipRight])//若左髋关节舵机不为异常值
                {
                    if(position[(int)servos.HipRight] <= Constants.HipRight_threshold )//若右髋关节舵机小于阀值，右腿偏向右
                    {
                        if(position[(int)servos.ThighRight ] >= Constants.ThighRight_threshold)//若右大腿大舵机大于阀值，右腿偏向前
                        {
                            if (Math.Abs((position[(int)servos.HipRight ] - Constants.HipRight_threshold)) > Math.Abs((position[(int)servos.ThighRight]) - Constants.ThighRight_threshold))//向右分量大于向前分量
                            {
                                position[Constants.POSITION_LENTH - 1] = (int)pose.walk_right;//则右行
                                return;
                            }
                            else
                            {
                                position[Constants.POSITION_LENTH - 1] = (int)pose.walk_right;//则前行
                                return;
                            }                           
                        }
                        position[Constants.POSITION_LENTH - 1] = (int)pose.walk_right;//则右行
                        return;
                    }
                    else if(position[(int)servos.ThighRight ] >= Constants.ThighRight_threshold)//若右大腿大舵机大于阀值，右腿偏向前
                    {
                        position[Constants.POSITION_LENTH - 1] = (int)pose.walk_right;//则前行
                        return;
                    }
                }
                //else
                //{
                //    if (position[(int)servos.ThighRight ] >= Constants.ThighRight_threshold)//若右大腿大舵机大于阀值，右腿偏向前
                //    {
                //        position[Constants.POSITION_LENTH - 1] = (int)pose.walk_right;//则前行
                //        return;
                //    }
                //    else
                //    {
                //        position[Constants.POSITION_LENTH - 1] = (int)pose.stand_up;
                //        return;
                //    }
                //}                
            }
            position[Constants.POSITION_LENTH - 1] = (int)pose.stop;
        }
        /// <summary>
        /// 舵机位置枚举类型
        /// </summary>
        public enum servos
        {
            /// <summary>
            /// 左大腿
            /// </summary>
            ThighLeft = 0,

            /// <summary>
            /// 左膝
            /// </summary>
            KneeLeft = 1,

            /// <summary>
            /// 左踝
            /// </summary>
            AnkleLeft = 2,

            /// <summary>
            /// 右大腿
            /// </summary>          
            ThighRight = 3,

            /// <summary>
            /// 右膝
            /// </summary>
            KneeRight = 4,

            /// <summary>
            /// 右踝
            /// </summary>
            AnkleRight = 5,

            /// <summary>
            /// 左肩
            /// </summary>
            ShoulderLeft = 6,

            /// <summary>
            /// 右肩
            /// </summary>
            ShoulderRight = 7,

            /// <summary>
            /// 左髋
            /// </summary>
            HipLeft = 8,

            /// <summary>
            /// 左脚
            /// </summary>
            FootLeft = 9,

            /// <summary>
            /// 右髋
            /// </summary>
            HipRight = 10,

            /// <summary>
            /// 右脚
            /// </summary>
            FootRight = 11,

            /// <summary>
            /// 左肘
            /// </summary>
            ElbowLeft = 12,

            /// <summary>
            /// 左手
            /// </summary>
            HandLeft = 13,

            /// <summary>
            /// 右肘
            /// </summary>
            ElbowRight = 14,

            /// <summary>
            /// 右手
            /// </summary>
            HandRight = 15,

            /// <summary>
            /// 头
            /// </summary>
            Head = 16
        }

        /// <summary>
        /// 机器人动作枚举
        /// </summary>
        enum pose
        {
            /// <summary>
            /// 无动作
            /// </summary>
            stop = 0,
            /// <summary>
            /// 蹲
            /// </summary>
            squat = 1,
            /// <summary>
            /// 左行
            /// </summary>
            walk_left = 2,
            /// <summary>
            /// 右行
            /// </summary>
            walk_right = 3,
            /// <summary>
            /// 前行
            /// </summary>
            walk_front = 4,
            /// <summary>
            /// 起立
            /// </summary>
            stand_up = 5
        }
    }
}
