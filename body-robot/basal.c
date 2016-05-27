//基础子程序
#include <reg60s2.h>
#include <delay.h>
#include <wifi-module.h>
#include <shr-51-c.h>
#include <math.h> 
#include <adc.h>
#include <walking.h>

uchar  position[24]={0};             //用于记录24个舵机的位置
/*
        数组对应端口说明：
         position[0]       ━━━━━>>      P0.0          x1舵机  x轴            
         position[1]       ━━━━━>>      P0.1          x2舵机                 
         position[2]       ━━━━━>>      P0.2          x3舵机                 
         position[3]       ━━━━━>>      P0.3          x4舵机                 
         position[4]       ━━━━━>>      P0.4          x5舵机                 
         position[5]       ━━━━━>>      P0.5          x6舵机                 
         position[6]       ━━━━━>>      P0.6          x7舵机                 
         position[7]       ━━━━━>>      P0.7          x8舵机
		                  
         position[8]       ━━━━━>>      P1.0          y1舵机  y轴        
         position[9]       ━━━━━>>      P1.1          y2舵机               
         position[10]      ━━━━━>>      P1.2          y3舵机                 
         position[11]      ━━━━━>>      P1.3          y4舵机                 
         position[12]      ━━━━━>>      P1.4          y5舵机                 
         position[13]      ━━━━━>>      P1.5          y6舵机                 
         position[14]      ━━━━━>>      P1.6          y7舵机                 
         position[15]      ━━━━━>>      P1.7          y8舵机    
		              
         position[16]      ━━━━━>>      P2.0	       z1舵机  z轴
         position[17]      ━━━━━>>      P2.1	       z2舵机
         position[18]      ━━━━━>>      P2.2		   z3舵机
         position[19]      ━━━━━>>      P2.3		   z4舵机
         position[20]      ━━━━━>>      P2.4		   z5舵机
         position[21]      ━━━━━>>      P2.5		   z6舵机
         position[22]      ━━━━━>>      P2.6		   z7舵机
         position[23]      ━━━━━>>      P2.7		   z8舵机
*/
uchar position_initial[24];
int position_change[24]; 
sint16 jichu[24];

uchar pick_up[8];           //kouchu[8];各个io口轮流输出是屏蔽其它位
uchar arr[8];               //提供排序空间 paixu_ncha[8]=0;      //提供排序空间

uchar t0bit=0;			    //定时器0周期相同标志位
uchar total;			    //千手观音总共的台数M
uchar part;				    //本机所属台数N
uchar change;			    //每个动作的变化量S

//#############################################################################
// 函数名称：low_level_500u(uchar time)
// 函数说明：PWM信号低电平时间子程序，控制舵机PWM信号的低电平时间决定舵机转动的速度
// 入口参数：低电平时间，500u为基础
// 出口参数：无
//#############################################################################
void low_level_500u(sint16 time)
{
  sint16 i; 
  for(i=0;i<time;i++)
  {delay500us(1);}
}
//#############################################################################
// 函数名称：void t0_init(void)
// 函数说明：t0定时器初始化，用于舵机2.5ms定时
// 入口参数：无
// 出口参数：无
//#############################################################################
void t0_init(void)
{
  TMOD  = TMOD & 0xf0;        //初始化定时器1的计数方式为方式1
  TMOD  = TMOD | 0x01;

  TH0=0xf7;			          //22.1184MHz,2.5Ms定时dc00
  TL0=0x00;
  ET0=1;						
  EA=1;
  t0bit=1;
}
//#############################################################################
// 函数名称：low_level_t0(uchar TH,uchar TL)
// 函数说明：同周期定时器0设置及启动程序，使每变化一个变化量的周期相同
// 入口参数：THTL定时器初值。
// 出口参数：无
//#############################################################################
 void low_level_t0(uint THTL)
{
  TH0=THTL>>8;			  //22.1184MHz,2.5Ms定时dc00
  TL0=THTL;
  t0bit=0;
  TR0=1;
}
//#############################################################################
// 函数名称：void T0_Interrupt(void) interrupt 1
// 函数说明：PWM信号低电平时间子程序，控制舵机PWM信号的低电平时间决定舵机转动的速度
// 入口参数：低电平时间，500u为基础
// 出口参数：无
//#############################################################################	 
void T0_Interrupt(void) interrupt 1
{  TH0=0xdc;		     //22.1184MHz,2.5Ms定时dc00
   TL0=0x00;
   t0bit=1;
}
//#############################################################################
// 函数名称：void ForwardDelay()
// 函数说明：千手观音不同步前延时子程序
// 入口参数：无
// 出口参数：无
//#############################################################################
void ForwardDelay(void)
{
		uchar  i;
		for(i=0;i<(part-1)*change;i++)
		  {
		  delay10ms(1);	 
		  delay10ms(1);	 
		  }	  
}
//#############################################################################
// 函数名称：void BackDelay()
// 函数说明：千手不同步后延时子程序
// 入口参数：无
// 出口参数：无
//#############################################################################

void BackDelay(void)
{
		uchar  i;		
		  for(i=0;i<(total-part)*change;i++)
		  {
		  delay10ms(1);	 
		  delay10ms(1);	 
		  }	  
}
//#############################################################################
// 函数名称：void array( )
// 函数说明：排序子程序，将各个口的8位根据时间的长短排序
// 入口参数：无
// 出口参数：无
//#############################################################################
void array()
{ 
	uchar i=0,j=0,x=0;
	     pick_up[0]=0xFE;
         pick_up[1]=0xFD;
         pick_up[2]=0xFB;
         pick_up[3]=0xF7;
         pick_up[4]=0xEF;
         pick_up[5]=0xDF;
         pick_up[6]=0xBF;
         pick_up[7]=0x7F;
	//排序
	for(i=0;i<=6;i++)
	{         for(j=i+1;j<=7;j++)
         {
                if(arr[i]<arr[j])
                  {             
				            
                           x=arr[j];
						   arr[j]=arr[i];
						   arr[i]=x;
					  
					       x=pick_up[j];
						   pick_up[j]=pick_up[i];
                           pick_up[i]=x;  
                  }
         }}	
	for(i=0;i<=6;i++)
         {
                   arr[i]= arr[i]- arr[i+1];
         } 		
}
//#############################################################################
// 函数名称：int max(uchar total，int M)
// 函数说明：求数组中绝对值最大的元素的子程序
// 入口参数：无
// 出口参数：无
//#############################################################################
int max(uchar total,int M)
{ uchar i;
	//找最大
	for(i=0;i<total;i++)
	{   if (abs(position_change[i])>M)
	        M=abs(position_change[i]);
	}	
		return M;			 
}
//#############################################################################
//函数名称：void p_to_p(int delay)
//函数说明：position 变化子程序，实现同起同落
//入口参数：delay=延时，define:p-to-p决定循环次数由自己决定还是通过程序算出结果决定
//出口参数：无
//#############################################################################
void p_to_p(int delay,int define)
{
   uchar i,j;
   int a,b=0,M=0;
   float c;
   uchar position_buffer[24];	        

   for(i=0;i<24;i++)
   {
   	  position_buffer[i]=position[i];
   }   
   if(define==0)					 //决定循环次数M
   {
     M=max(24,M);
	 if(M>20)
     M=20;
   }   
   else
   	{M=define; 	 					 //循环次数决定完毕
	}
	 for(i=1;i<=M;i++)				 //开始循环M次输出
   {
      for(j=0;j<24;j++)
      {  c=position_change[j];
         if(i==1)		  
	        a=(c*i)/M;		
		 else
		 {
		    a=(c*i)/M;
			b=(c*(i-1))/M;
		 }
         position[j]+=(a-b);
	}
	PWM_24();
    low_level_500u(delay);
       }  				 		  		 
   for(i=0;i<24;i++) 		  		 //最后补偿每个舵机的误差  		 
   {
      position[i]=position_buffer[i]+position_change[i];
      position_change[i]=0;
   }
  PWM_24();
  low_level_500u(30);   
}

//##############################################################################
//函数名称：void relative(int a1,int a2,int a3,int a4,int a5,int a6)
//函数说明：修正24个舵机的变化量
//入口参数：无
//出口参数：无
//##############################################################################
void relative(int a1,int a2,int a3,int a4,int a5,int a6,int a7,int a8,int a9,\
              int a10,int a11,int a12,int a13,int a14,int a15,int a16,int a17)
{
  int *p;
  unsigned char i;
  p=&a1;
  
  		  
  for(i=0;i<17;i++,p++)
  {
	  position_change[i]=*p-position[i]+position_initial[i];
  }
  i=a2;i=a3;i=a4;i=a5;i=a6;i=a7;i=a8;i=a9;i=a10;i=a11;i=a12;i=a13;i=a14;i=a15;i=a16;i=a17;			  //无特殊意义
}

//#############################################################################
// 函数名称：void initial_position(void)
// 函数说明：初始位置子程序，根据各个舵机的不同位置设置初始位置。
// 入口参数：无
// 出口参数：无
//#############################################################################
void initial_position(void)                 
{        uchar i;
	     for(i=0;i<30;i++)
         {	     
						 position[0]=145;       //140
             position[1]=52;    //62
             position[2]=129;   //140
             position[3]=105;   //110
             position[4]=189;    //189
             position[5]=121;    //111
             position[6]=224;    //216
             position[7]=24;     //15
             //PB口
             position[8]=122;      //95
             position[9]=116;      //111
             position[10]=129;     //131
             position[11]=120;     //113
             position[12]=218;      //206
             position[13]=114;      //114
             position[14]=34;       //30
             position[15]=119;			 	//19
			 //PC口
             position[16]=125;   //125
			 position[17]=125;
             position[18]=125;
     		 position[19]=125;                      
						   
			 position[20]=125;
             position[21]=125;
                           
		     position[22]=125;
             position[23]=125;
		  			
		     PWM_24();
             low_level_500u(10);       
         }                       
         for(i=0;i<24;i++)
		 {
		     position_initial[i]=position[i];
			 position_change[i]=0;
			 jichu[i]=0;
		 }          			   		               	  
	
} 	

//#############################################################################
// 函数名称：void initial_position(void)
// 函数说明：初始位置子程序，根据各个舵机的不同位置设置初始位置。
// 入口参数：无
// 出口参数：无
//#############################################################################
void initial_position_slow(void)                 
{        
    uchar i;
	                          
    for(i=0;i<24;i++)
    {
        position_change[i]=position_initial[i]-position[i];
	}  
	        			   		               	  
	p_to_p(0,30);
} 	
   
//#############################################################################
//函数名称：void sit_down(uchar step)
//函数说明：蹲下子程序，使机器人处于微蹲状态，便于行走。
//入口参数：setp, 表示下蹲的程度。
//出口参数：无
//#############################################################################
void sit_down(uchar step)
{        
         uchar i;
	 
         for(i=0;i<step;i++)
		    {
			   position[0]--;              //左腿下蹲
			   position[1]+=2;             
			   position[2]++;               
			   
               position[3]++;              //右腿下蹲
			   position[4]-=2;              
			   position[5]--;               
			   PWM_24();
               low_level_500u(30);
			}			
}
//#############################################################################
//函数名称： void stand_up(uchar setp)
//函数说明：起立子程序，有些动作需在站直的情况下做
//入口参数：setp, 表示起立的程度。
//出口参数：无
//#############################################################################
void stand_up(uchar setp)
{        
         uchar i;
		
         for(i=0;i<setp;i++)
		    {
			    position[0]++;             //左褪起立
			    position[1]-=2;            
			    position[2]--;             

                position[3]--;             //右腿起立
			    position[4]+=2;           
			    position[5]++;            
			    PWM_24();
                low_level_500u(20);       		
			}
}	
//#############################################################################
//函数名称： void turn_left(uchar step)
//函数说明：左转子程序
//入口参数：step, 表示左转次数。
//出口参数：无
//#############################################################################

void turn_left(uchar step)
{   uchar j,i;  
               sit_down(50);
		       delay500ms(1);
			  
		  for(i=0;i<step;i++)                     //左侧身                           
			{	   
			  for(j=0;j<20;j++)                     //左侧身                           
			    {
			     position[8]-=1;
				 position[9]+=1;
				 position[10]-=1;
				 position[11]+=1;
				 PWM_24();
                 low_level_500u(5);								
                }
			  for(j=0;j<20;j++)                    //抬右腿                     
			    {
			     position[3]+=1;
				 position[4]-=2;
				 position[5]-=1;
				   PWM_24();
                 low_level_500u(5);								
                }	
			  for(j=0;j<20;j++)                    //迈右腿               
			    {
			     position[0]+=1;
				 position[2]+=1;
				 position[3]+=1;
				 position[5]+=1;
				   PWM_24();
                 low_level_500u(5);								
                }
			  for(j=0;j<20;j++)                     //放右腿                     
			    {
			     position[3]-=1;
				 position[4]+=2;
				 position[5]+=1;
				   PWM_24();
                 low_level_500u(5);								
                }
			  for(j=0;j<20;j++)                     //                           
			    {
			     position[8]+=1;
				 position[9]-=1;
				 position[10]+=1;
				 position[11]-=1;
				 PWM_24();
                 low_level_500u(5);								
                }
			  for(j=0;j<20;j++)                                   
			    {
			     position[0]-=1;
				 position[2]-=1;
				 position[3]-=1;
				 position[5]-=1;
				   PWM_24();
                 low_level_500u(5);								
                }
			}
			 
			
		    stand_up(50);
} 
//#############################################################################
//函数名称：void turn_left(uchar step)
//函数说明：右转子程序
//入口参数：step, 表示右转次数。
//出口参数：无
//#############################################################################
 void turn_right(uchar step)
{       uchar j,i;    
           sit_down(50);
		     delay500ms(1);
		for(i=0;i<step;i++)                                              
		   {
			 for(j=0;j<20;j++)                    //右侧身                         
			    {
			     position[8]+=1;
				 position[9]-=1;
				 position[10]+=1;
				 position[11]-=1;
				 PWM_24();
                 low_level_500u(5);								
                }
			  for(j=0;j<20;j++)                    //抬左腿                     
			    {
			     position[0]-=1;
				 position[1]+=2;
				 position[2]+=1;
				   PWM_24();
                 low_level_500u(5);								
                }
			  for(j=0;j<20;j++)                    //迈左腿               
			    {
			     position[0]-=1;
				 position[2]-=1;
				 position[3]-=1;
				 position[5]-=1;
				   PWM_24();
                 low_level_500u(5);								
                }
			  for(j=0;j<20;j++)                     //放左腿                     
			    {
			     position[0]+=1;
				 position[1]-=2;
				 position[2]-=1;
				   PWM_24();
                 low_level_500u(5);								
                }
			  for(j=0;j<20;j++)                                                
			    {
			     position[8]-=1;
				 position[9]+=1;
				 position[10]-=1;
				 position[11]+=1;
				 PWM_24();
                 low_level_500u(5);								
                }
			  for(j=0;j<20;j++)                                 
			    {
			     position[0]+=1;
				 position[2]+=1;
				 position[3]+=1;
				 position[5]+=1;
				   PWM_24();
                 low_level_500u(5);								
                }
			}
		
			 delay500ms(1);            
			 stand_up(50);
 }

//#############################################################################
//函数名称： void l_pyi(uchar step)
//函数说明：左平移子程序
//入口参数：step, 表示左平移次数。
//出口参数：无
//#############################################################################
void l_pyi(uchar step)
{
       uchar j,i;            
         sit_down(20);
		delay500ms(1);
        for(i=0;i<step;i++)
		  {
	        for(j=0;j<15;j++)             
		       {
			    position[8]++;              
			    position[9]--;             

                position[10]++;             
			    position[11]--;          

                PWM_24();
                    low_level_500u(30);
		
			   }  
             for(j=0;j<10;j++)                 
		       {
               position[0]--;                    
			   position[1]++;               
               position[1]++;
			   position[2]++;               
			   position[8]++;               
			   position[9]--;               
             PWM_24();
                    low_level_500u(30);
		      }

			for(j=0;j<10;j++)                
		      {
               position[0]++;               
			   position[1]--;
               position[1]--;
			   position[2]--;
			   position[8]++;               
			   position[9]--;               
        PWM_24();
                    low_level_500u(30);
		       }

            for(j=0;j<50;j++)             
		       {
			    position[8]--;            
			    position[9]++;            

                position[10]--;           
			    position[11]++;           
            PWM_24();
                    low_level_500u(30);
   		        }
			 for(j=0;j<20;j++)             
		        {
			    position[10]++;            
			    position[11]--;            
             PWM_24();
                    low_level_500u(30);
   		        }
			 for(j=0;j<15;j++)                  
		        {
			    position[8]++;            
			    position[9]--;           

                position[10]++;          
			    position[11]--;           
              PWM_24();
                    low_level_500u(30);
			   }
           }delay500ms(1);
	        stand_up(20);
			
}			
//#############################################################################
//函数名称： void r_pyi(uchar step)
//函数说明：右平移子程序
//入口参数：step, 表示右平移次数。
//出口参数：无
//#############################################################################
void r_pyi(uchar step)
{
    uchar j,i;
             
          sit_down(20);
		 delay500ms(1);
        for(i=0;i<step;i++)
		  {
	        for(j=0;j<15;j++)                  
		       {
			    position[8]--;           
			    position[9]++;           

                position[10]--;           
			    position[11]++;          

             PWM_24();
                    low_level_500u(30);
			  
		
			   }  
             for(j=0;j<10;j++)                 
		       {
               position[3]++;                    
			   position[4]--;               
               position[4]--;
			   position[5]--;               
			   position[10]--;               
			   position[11]++;             
              
			PWM_24();
                    low_level_500u(30);
		      }

			for(j=0;j<10;j++)                
		      {
               position[3]--;               
			   position[4]++;
               position[4]++;
			   position[5]++;
			   position[10]--;               
			   position[11]++;             
              
			PWM_24();
                    low_level_500u(30);
		       }

            for(j=0;j<50;j++)             
		       {
			    position[8]++;            
			    position[9]--;            

                position[10]++;          
			    position[11]--;           
              PWM_24();
                    low_level_500u(30);
			
   		        }
			 for(j=0;j<20;j++)            
		        {
			    position[8]--;            
			    position[9]++;            
               PWM_24();
                    low_level_500u(30);
			    
   		        }
			 for(j=0;j<15;j++)                  
		        {
			    position[8]--;            
			    position[9]++;            

                position[10]--;           
			    position[11]++;          

                PWM_24();
                    low_level_500u(30);
			   }
           } delay500ms(1);
		    stand_up(20);
			   
}		  
//#############################################################################
//函数名称： void fwc_paxia()
//函数说明：俯卧撑趴下，俯卧撑之前的趴下动作，与其它动作合用
//入口参数：无
//出口参数：无
//#############################################################################
void fwc_paxia(void)
{
      uchar i;
   
	      for(i=0;i<40;i++)
		     {
			   position[0]-=3;              //  弯腿弯腰
			   position[1]--;               
			   position[2]-=2;              
               position[3]+=3;               
			   position[4]++;                
			   position[5]+=2;              
			   position[6]-=2;              
               position[7]+=2;              
			   PWM_24();
                    low_level_500u(15);
		      }
			 
		   for(i=0;i<10;i++)
		     {
			  position[0]--;             
			  position[3]++;              
			  position[6]--;                  
              position[7]++;               
			  PWM_24();
                    low_level_500u(10);
		     }
		   for(i=0;i<60;i++)
		     {
			  position[2]++;              //  向前倾
			  position[5]--;               
			  position[6]-=2;             
              position[7]+=2;             
			  PWM_24();
                    low_level_500u(10);
		     }
			 
			  for(i=0;i<10;i++)
		    {
			  position[2]++;              //  向前倾
			  position[5]--;             
			    PWM_24();
                    low_level_500u(10);
		     }
	  	for(i=0;i<80;i++)
		    {
			   position[0]+=2;             
			   position[3]-=2;               
			   position[6]++;               //手向后划
    	       position[7]--;                
		      PWM_24();
                    low_level_500u(10);
		     }
}									  
//#############################################################################
//函数名称： void fwc_qilai()
//函数说明：俯卧撑起来，俯卧撑起来动作，与其它程序共用
//入口参数：无
//出口参数：无
//#############################################################################
void fwc_qilai(void)
{
         uchar i;
          for(i=0;i<10;i++)
		    {
			   position[0]--;               
    	       position[3]++;               
		       PWM_24();
               low_level_500u(10);
		     }
		  for(i=0;i<80;i++)
		    {
			   position[0]-=2;              
               position[3]+=2;             
               position[6]-=1;              // 手向前划
    	       position[7]+=1;             
		       PWM_24();
               low_level_500u(10);
		     }
		for(i=0;i<30;i++)
		    {
			  position[2]--;                //  向后倾
			  position[5]++;                
			  PWM_24();
                    low_level_500u(45);
		     }
	 for(i=0;i<60;i++)
		    {
			  position[2]--;               //  向后倾
			  position[5]++;              
			  position[6]+=2;             
              position[7]-=2;             
			  PWM_24();
              low_level_500u(20);
		     }
	 for(i=0;i<10;i++)
		    {
			  position[0]++;              
			  position[3]--;             
			  PWM_24();
              low_level_500u(30);
		     }
      for(i=0;i<40;i++)
		    {
			   position[0]+=2;             // 逆弯腿弯腰
			   position[1]++;              
			   position[2]+=2;             
	           position[3]-=2;              
			   position[4]--;               
			   position[5]-=2;             
			   PWM_24();
               low_level_500u(25);
		     }
		 for(i=0;i<5;i++)
		    {  
               position[0]+=2;	
			   position[3]-=2;
			   PWM_24();
               low_level_500u(15);
		     }
		
		       initial_position();               //调用机器人初始位置子程序
			 delay10ms(100);                        
		
}

//#############################################################################
//函数名称： void fwc(uchar step)
//函数说明：俯卧撑子程序
//入口参数：step, 表示俯卧撑次数。
//出口参数：无
//#############################################################################
void fwc(uchar step)
{
      uchar i,j;
	  
		       initial_position();               //调用机器人初始位置子程序
         fwc_paxia();
		
            for(i=0;i<30;i++)
		      {
			   position[6]++;              
    	       position[7]--;             
		      PWM_24();
                    low_level_500u(10);
		       }
             for(j=0;j<step;j++)           //俯卧撑
		     {
			  for(i=0;i<30;i++)
		      {
			   position[12]-=2;           
			   position[13]+=3;           
			   position[14]+=2;           
    	       position[15]-=3;           
		      PWM_24();
                    low_level_500u(10);
		      }
			for(i=0;i<30;i++)
		      {
			   position[12]+=2;            
			   position[13]-=3;            
			   position[14]-=2;            
    	       position[15]+=3;          
		       PWM_24();
                    low_level_500u(10);
		      }
		   }
		      fwc_qilai();
}


//#############################################################################
//函数名称： void goal_l(uchar step)
//函数说明：左脚踢球(站立)子程序
//入口参数：step, 表示踢球次数。
//出口参数：无
//#############################################################################
void goal_l(uchar step)
{ uchar i;
       sit_down(44);
		 delay500ms(1);
	  while(step)
       {  
	    
	     step--;
         for(i=0;i<8;i++)            //双手分开
			   {
			        position[12]--; 
					position[14]++;  
                    PWM_24();
                    low_level_500u(10);
			   }

        for(i=0;i<24;i++)
		    {
			    position[8]++;            
			    position[9]--;            

                position[10]++;         
			    position[11]--;           
				position[12]--;        
                position[14]--;          
             
			    PWM_24();
	   		    low_level_500u(25);
                                
			}
			
			for(i=0;i<16;i++)       //抬左腿
               {
			        position[0]--;  
					position[1]+=2;  
                  	position[2]++;  
					PWM_24();
                    low_level_500u(10);
			   }
 
            for(i=0;i<16;i++)      //躬身//
               {			      
					position[0]++;  
                  	position[3]++;  
					PWM_24();
                    low_level_500u(10);
			   }

		                           //射门
            position[0]=position[0]-24;  
           	position[3]=position[3]-24;  
            PWM_24();

			for(i=0;i<1;i++)      //左腿加力
               {
                  	position[0]=position[0]-38;  
					PWM_24();
                    low_level_500u(10);
			   }

			for(i=0;i<19;i++)      //左腿收回
               {
                  	position[0]+=2; 
					PWM_24();
                    low_level_500u(10);
			   }

			for(i=0;i<8;i++)      //回身
               {
                  	position[0]++;  
					position[3]++;  
					PWM_24();
                    low_level_500u(10);
			   }

		    for(i=0;i<16;i++)      //落左腿
               {
                  	position[0]++;  
					position[1]-=2;  
			    	position[2]--;  
					PWM_24();
                    low_level_500u(10);
			   }
			 for(i=0;i<24;i++)
		    {
			    position[8]--;            
			    position[9]++;           

                position[10]--;         
			    position[11]++;          
				position[12]++;              
                position[14]++;          
         		   
			    PWM_24();
			  	low_level_500u(20);            
              }
            for(i=0;i<8;i++)         //双手合拢//
			   {
			        position[12]++;  
					position[14]--;  
                    PWM_24();
                    low_level_500u(10);
			   }           
    	 }
		  delay500ms(1);
		  stand_up(44);
}


//#############################################################################
//函数名称： void goal_r(uchar step)
//函数说明：右脚踢球(站立)子程序
//入口参数：step, 表示踢球次数。
//出口参数：无
//#############################################################################
void goal_r(uchar step)
 {
		uchar i=0;
		sit_down(44);
		delay500ms(1);
        while(step) 
 	      {
		    step--;
			for(i=0;i<8;i++)         //双手分开
			   {
			        position[12]--;  
					position[14]++;  
                    PWM_24();
                    low_level_500u(10);
			   }
             for(i=0;i<24;i++)		 
		        {
			    position[8]--;            
			    position[9]++;           

                position[10]--;         
			    position[11]++;          
				position[12]++;              
                position[14]++;        
			   
			    PWM_24();
			  	low_level_500u(25);          
                
     		}
          		
			for(i=0;i<16;i++)        //抬右腿
               {
			        position[3]++; 
					position[4]-=2;  
                  	position[5]--;  
					PWM_24();
                    low_level_500u(10);
			   }

            for(i=0;i<16;i++)       //躬身
               {
			      
					position[0]--;  
                  	position[3]--;  
					PWM_24();
                    low_level_500u(10);
			   }

			   //射门
            position[0]=position[0]+24;   
           	position[3]=position[3]+24;  
            PWM_24();

			for(i=0;i<1;i++)       //右腿加力
               {
                  	position[3]=position[3]+38;  
					PWM_24();
                    low_level_500u(10);
			   }

			for(i=0;i<19;i++)      //右腿收回
               {
                  	position[3]--; 
					position[3]--;  
					PWM_24();
                    low_level_500u(10);
			   }

			for(i=0;i<8;i++)      //回身
               {
                  	position[0]--;  
					position[3]--;  
					PWM_24();
                    low_level_500u(10);
			   }

		    for(i=0;i<16;i++)      //落右腿//
               {
                  	position[3]--;  
					position[4]+=2;  
			    	position[5]++;  
					PWM_24();
                    low_level_500u(10);
			   }
			 for(i=0;i<24;i++)
		    {
			    position[8]++;            
			    position[9]--;            

                position[10]++;         
			    position[11]--;           
				position[12]--;        
                position[14]--;          
             
			    PWM_24();
	   		    low_level_500u(20);
                
			}
             for(i=0;i<8;i++)         //双手合拢//
			   {
			        position[12]++;   
					position[14]--;   
                    PWM_24();
                    low_level_500u(10);
			   }
		}
		 delay500ms(1);
			  stand_up(44);
}
//#############################################################################
//函数名称： void qianpaxia(void)
//函数说明：前趴下子程序
//入口参数：无
//出口参数：无
//#############################################################################
void qianpaxia(void)
{
           uchar i;
               fwc_paxia();
		     for(i=0;i<65;i++)
		       {
			    position[6]+=2;              // 手向后划
    	        position[7]-=2;              
				PWM_24();
                low_level_500u(30);
		       }
			     delay1s(1);
				 delay1s(1);
}
//#############################################################################
//函数名称： void qianpq(void)
//函数说明：前爬起子程序
//入口参数：无
//出口参数：无
//#############################################################################

void qianpq(void)
{
    //------------------------------------------------------------------------------
	relative(0,0,0,0,0,0,0,0,0,0,0,0,-80,0,80,0,0);
	p_to_p(0,10);
	
	relative(0,0,0,0,0,0,-100,100,0,0,0,0,-80,0,80,0,0);
	p_to_p(0,20);
   //------------------------------------------------------------------------------
	relative(0,-10,0,0,4,0,-85,86,0,0,0,0,0,0,0,0,0);
    p_to_p(0,20);
   
	relative(-90,-30,6,90,24,-10,-150,150,0,0,0,0,0,0,0,0,0);
    p_to_p(0,20);
	
	relative(-110,-50,-40,110,44,40,-150,150,0,0,0,0,0,0,0,0,0);
    p_to_p(0,10);
	
	relative(-110,-50,-70,110,44,70,-150,150,0,0,0,0,0,0,0,0,0);
    p_to_p(0,10);
	
	relative(0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0);
	p_to_p(0,30);
	delay10ms(10);	 
}
//#############################################################################
//函数名称： void hp(void)
//函数说明：后趴下子程序
//入口参数：无
//出口参数：无
//#############################################################################
void hp(void)
{
    uchar i;  	
  for (i=0;i<60;i++)
  {
    position[6]-=2;     
    position[7]+=2;          
    PWM_24();
    low_level_500u(20);
  }

  for (i=0;i<50;i++)
  {
    position[12]-=4;     
    position[14]+=4;          
     PWM_24();
    low_level_500u(20);
  }

  for (i=0;i<55;i++)
  {
    position[0]-=2;
    position[1]+=3;
    position[2]++;
	position[3]+=2;
    position[4]-=3;  
    position[5]--;
   PWM_24();
    low_level_500u(20);
  }   
 
  for (i=0;i<110;i++)
  {
    position[0]++;     
    position[3]--;          
     PWM_24();
    low_level_500u(10);
  }


  for (i=0;i<60;i++)
  {
    position[6]+=2;     
    position[7]-=2;          
     PWM_24();
    low_level_500u(20);
  }
initial_position(); 
}
//#############################################################################
//函数名称：void hp(void)
//函数说明：后爬起子程序
//入口参数：无
//出口参数：无
//#############################################################################
void hpq(void)
{
    relative(-70,106,-54,70,-110,55,-114,120,0,0,0,0,-169,0,164,0,0);
	p_to_p(0,20);
	
	relative(-70,146,5,70,-150,0,-114,120,0,0,0,0,-159,0,154,0,0);
	p_to_p(0,20);
	
	relative(60,113,87,-60,-120,-91,-103,109,0,0,0,0,-182,0,179,0,0);
	p_to_p(0,20);
	
	relative(-70,143,78,70,-149,-81,0,0,0,0,0,0,0,0,4,0,0);
	p_to_p(0,40);
	
	relative(0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0);
	p_to_p(0,30);
	delay10ms(10);
}
//#############################################################################
//函数名称： void yangwoqizuo(uchar step)
//函数说明：仰卧起坐子程序
//入口参数：step, 表示仰卧起坐的次数。
//出口参数：无
//#############################################################################

void yangwoqizuo(uchar step)
{
         uchar i,j;
            hp();
			delay1s(1);
	      for(i=0;i<5;i++)
		    {
			 position[8]--;              //  双腿合拢 便于弯腰
			 position[9]++;               
			 position[10]++;             
			 position[11]--;              
			  PWM_24();
                    low_level_500u(30);
		     }
		  for(i=0;i<40;i++)
		      {
			   position[6]-=4;               
               position[7]+=4;               
                 PWM_24();
    low_level_500u(20);
		      }
		  for(i=0;i<10;i++)                   
		      {
			   position[0]-=4;               
               position[2]+=4;
               position[3]+=4;               
               position[5]-=4;
			   position[12]+=1;               
               position[13]+=10;
               position[14]-=1;               
               position[15]-=10;
               PWM_24();
               low_level_500u(20);
			   delay1ms(50);
				
		      }
			  for(j=0;j<step;j++)
		   {
			for(i=0;i<40;i++)
		      {
			   position[0]-=2;               
               position[3]+=2;               
               PWM_24();
               low_level_500u(40);
		      }
			for(i=0;i<40;i++)
		      {
			   position[0]+=2;               
               position[3]-=2;               
               PWM_24();
               low_level_500u(40);
		      }
		   }
             delay1s(1);
		     initial_position();               //调用机器人初始位置子程序
			 hpq();                     
}

//#############################################################################
//函数名称：void jingli_l(void)
//函数说明：敬礼（左手）子程序
//入口参数：无
//出口参数：无
//#############################################################################
void jingli_l(void)
{
      uchar i;
   stand_up(6);
  for(i=0;i<20;i++)
		    {
			   position[6]-=3;  
           	   position[13]+=3;
            			   
			  PWM_24();
                    low_level_500u(30);
		     }
			
	  for(i=0;i<30;i++)
		    {
			   position[0]--;              
			   position[3]++;                 
			  PWM_24();
                    low_level_500u(30);
		     }delay500ms(1);
		 for(i=0;i<30;i++)
		    {
			   position[0]++;              
			   position[3]--;                 
			 PWM_24();
                    low_level_500u(30);
		     }	 delay500ms(1);
   for(i=0;i<20;i++)
		    {
			   position[6]+=3;  
           	   position[13]-=3;
            
			PWM_24();
                    low_level_500u(30);
		     }
	
	 sit_down(6);
    delay500ms(1);
  }
//#############################################################################
//函数名称： void jingli_r(void)
//函数说明：敬礼（右手）子程序
//入口参数：无
//出口参数：无
//#############################################################################

void jingli_r(void)
{
      uchar i;
    stand_up(6);
   for(i=0;i<20;i++)
		    {
			   position[7]+=3;  
           	   position[15]-=3;
          			
			  PWM_24();
                    low_level_500u(30);
        	
		     }
			  
	  for(i=0;i<30;i++)
		    {
			   position[0]--;             
			   position[3]++;               
			 PWM_24();
                    low_level_500u(30);
		     }
      delay1s(1);

  	  for(i=0;i<30;i++)
		    {
			   position[0]++;             
			   position[3]--;             
			PWM_24();
                    low_level_500u(30);
		     }
	 for(i=0;i<20;i++)
		    {
			   position[7]-=3;  
			   position[15]+=3;
 			 PWM_24();
                    low_level_500u(30);
		     }
	 sit_down(6);
      delay1s(1);
  }

//#############################################################################
//函数名称： void qgf(void)
//函数说明：前滚翻子程序
//入口参数：step, 表示前滚翻次数。
//出口参数：无
//#############################################################################
void qgf(void)
{

    relative(-110,-50,-70,110,44,70,-130,130,0,0,0,0,0,0,0,0,0);
	p_to_p(0,50);
	
	relative(-30,-50,19,30,44,-20,-80,81,0,0,0,0,0,0,0,0,0);
	p_to_p(0,40);
	
	relative(-30,-50,19,30,44,-20,-60,61,0,0,0,0,0,0,0,0,0);
	p_to_p(0,20);

	relative(100,70,9,-100,-75,-10,-90,91,0,0,0,0,0,0,0,0,0);
    p_to_p(0,60);
	
	relative(100,70,9,-100,-75,-10,-150,151,0,0,0,0,0,0,0,0,0);
	p_to_p(0,30);
	
	relative(100,70,9,-100,-75,-10,-40,41,0,0,0,0,0,0,0,0,0);
	p_to_p(0,20);
	
	relative(100,70,9,-100,-75,-10,-40,41,0,0,0,0,-110,0,100,0,0);
	p_to_p(0,20);
	
	relative(90,90,73,-90,-95,-78,-90,91,0,0,0,0,-160,0,150,0,0);
	p_to_p(0,20);
	
	relative(-20,100,73,22,-105,-79,0,0,0,0,0,0,-10,0,0,0,0);
	p_to_p(0,60);
	
	relative(0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0);
	p_to_p(0,50);		  
} 
//#############################################################################
//函数名称： void hgf(void)
//函数说明：后滚翻子程序
//入口参数：step, 表示后滚翻次数。
//出口参数：无
//#############################################################################
void hgf(void)
{
              uchar i;
			
	  initial_position();                  //调用机器人初始位置设定第1族子程序
		  	for(i=0;i<5;i++)
		      {
			   position[8]--;              //   双腿合拢 便于弯腰
			   position[9]++;                
			   position[10]++;             
			   position[11]--;               
			   PWM_24();
    low_level_500u(10);
		      }
	        for(i=0;i<20;i++)
		      {
			   position[0]+=5;              //  弯腿弯腰
			   position[1]+=3;              
			   position[2]+=3;               
               position[3]-=5;                
			   position[4]-=3;               
			   position[5]-=3;                
			   position[6]-=6;         
               position[7]+=6;          
			   position[12]-=8;         
               position[14]+=8;          
			PWM_24();
                    low_level_500u(25);
		      }
			for(i=0;i<30;i++)
		      {
			   position[2]-=2;               
               position[5]+=2;                
			     PWM_24();
                    low_level_500u(5);
		      }
             for(i=0;i<50;i++)		  		  	
		      {
			   position[0]-=2;               
               position[3]+=2;                
			   PWM_24();
                    low_level_500u(5);
		      } 
			  
			for(i=0;i<60;i++)
		      {
			   position[0]-=2;
               position[1]-=2;
               position[3]+=2;
               position[4]+=2; 
			      PWM_24();
                    low_level_500u(5);
		      }
    		for(i=0;i<20;i++)
		      {
			   position[6]+=5;               
               position[7]-=5;                
			     PWM_24();
                    low_level_500u(5);
		      } 
		  for (i=0;i<35;i++)                    
		  {
		  position[6]-=2; 
		  position[7]+=2;
		  position[12]+=2;
		  position[14]-=2;
		  		 PWM_24();
                    low_level_500u(5);   	  
		  }
			   for (i=0;i<40;i++)
		  {
		  position[2]-=1;
		  position[5]+=1;
		  position[12]+=2;
		  position[14]-=2;
		  		 PWM_24();
                    low_level_500u(5);   	  
		  }
		  
	for(i=0;i<40;i++)
		    {
			  position[2]--;               // 向后倾
			  position[5]++;                
			  position[6]+=2;              
              position[7]-=2;              
			  PWM_24();
              low_level_500u(5);
		     }
	relative(0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0);
	p_to_p(0,50);  		   	     
}          
//#############################################################################
// 函数名称：void daoli(void)
// 功    能：倒立函数
// 入口参数：无
// 出口参数：无
//#############################################################################
void daoli ()
{ uchar i;	
for(i=0;i<20;i++)  
       {	position[0]-=5;	
		    position[2]-=1;	
			position[3]+=5;	
		    position[5]+=1;	
			PWM_24();
            low_level_500u(70);
	   }			  
delay1s(1);

for(i=0;i<20;i++)  
       {	position[2]+=3;	
		    position[5]-=3;	
			position[6]-=4;	
		    position[7]+=4;	
			PWM_24();
            low_level_500u(70);
	   }   
delay1s(1);


for(i=0;i<20;i++)  
       {	position[0]+=6;		   
			position[3]-=6;			    
			if ((i%2)==0)
			{;}
			else 
			{  position[2]-=3;
			   position[5]+=3;		
			   position[6]-=1;	
		       position[7]+=1;
			}
			PWM_24();
            low_level_500u(70);
	   }   
delay1s(4);
			
 for(i=0;i<20;i++)  
       {	position[0]-=5;	
		    position[2]+=1;	
			position[3]+=5;	
		    position[5]-=1;	
			if ((i%2)==0)
			{;}
			else 
			{
			position[6]+=1;	
		    position[7]-=1;	}
			PWM_24();
            low_level_500u(70);
		
	   }   
	  delay500ms(1);
  	for(i=0;i<10;i++)  		
	   {	
			position[12]-=5;
			position[14]+=5;		
			PWM_24();
            low_level_500u(70);
	   }   

	for(i=0;i<20;i++)  
       {	position[6]-=3;	
		    position[7]+=3;	
					
			PWM_24();
            low_level_500u(70);
	   }   
	
		for(i=0;i<10;i++)  
       {	position[2]-=2;	
		    position[5]+=2;	
			position[12]+=5;
			position[14]-=5;
					
			PWM_24();
            low_level_500u(70);
	   }   		
	   	for(i=0;i<10;i++)  
       {	position[1]-=4;	
		    position[4]+=4;	
								
			PWM_24();
            low_level_500u(70);
	   }   			     
	
		 	for(i=0;i<10;i++)  
       {	position[0]-=1;	
		    position[1]-=1;	
			position[2]-=2;	
		    position[3]+=1;	
			position[4]+=1;	
		    position[5]+=2;	
			position[6]+=1;
			position[7]-=1;		
			PWM_24();
            low_level_500u(70);
	   }  
	
	   	 	for(i=0;i<25;i++)  
       {	position[2]-=2;		    
		    position[5]+=2;				
			PWM_24();
            low_level_500u(100);
	   }  
	  
			 for(i=0;i<20;i++)  
       {	position[0]+=6;	
		    position[1]+=2;	
			position[2]+=3;	
		    position[3]-=6;	
			position[4]-=2;	
		    position[5]-=3;	
			position[6]+=6;
			position[7]-=6;		
			PWM_24();
            low_level_500u(70);
	   }    
	    
 initial_position();               //调用机器人初始位置子程序
}                                           	
//#############################################################################
// 函数名称：void pch(void)
// 功    能：横劈叉函数
// 入口参数：无
// 出口参数：无
//#############################################################################				 
void pch()
{ uchar i;

 for(i=0;i<19;i++)  
       {	
			position[8]+=5;
		    position[9]-=3;
			position[10]-=5;
			position[11]+=3;
			position[12]-=5;
			position[14]+=5;
					
			PWM_24();
            low_level_500u(30);
	   }  

delay1s(5);
for(i=0;i<20;i++)  
       {	
			position[0]-=4;
		    position[1]+=7;
			position[2]+=3;
			position[3]+=4;
			position[4]-=7;
			position[5]-=3;
					
			PWM_24();
            low_level_500u(30);
	   }  
delay1s(1);

for(i=0;i<19;i++)  
       {	
			position[8]-=5;
		    position[9]+=3;
			position[10]+=5;
			position[11]-=3;
					
			PWM_24();
            low_level_500u(90);
	   }  	 
	 	 for(i=0;i<20;i++)  
       {	
			position[0]+=4;
		    position[1]-=7;
			position[2]-=3;
			position[3]-=4;
			position[4]+=7;
			position[5]+=3;
					
			PWM_24();
            low_level_500u(60);
	   }  
	   initial_position();               //调用机器人初始位置子程序
}
//#############################################################################
// 函数名称：void dt(void)
// 功    能：单腿直立函数
// 入口参数：无
// 出口参数：无
//#############################################################################
void dt ()
{ uchar i;
		 
		 	for(i=0;i<40;i++)  
       {	
			position[12]-=2;
			position[14]+=2;		
			PWM_24();
            low_level_500u(30);
	   }  
	  

	for(i=0;i<20;i++)  
       {	
			position[8]+=1;
			position[9]-=1;
			position[10]+=1;
			position[11]-=1;		
			PWM_24();
            low_level_500u(80);
	   }  
  delay1s(1);


	for(i=0;i<20;i++)  
       {	
			position[8]+=4;
				if ((i%2)==0)
			{;}
			else 
			  { position[11]-=1; }
			
			PWM_24();
            low_level_500u(100);
	   }  
delay1s(5);
for(i=0;i<10;i++)  
       {	
			position[8]-=8;
			position[11]+=1;		
			PWM_24();
            low_level_500u(100);
	   }  
delay1s(1);
		   
 for(i=0;i<10;i++)  
       {	
			position[8]-=2;
		    position[9]+=2;
			position[10]-=2;
			position[11]+=2;		
			PWM_24();
            low_level_500u(70);
	   }  
delay1s(1);
 initial_position();               //调用机器人初始位置子程序
}
//#############################################################################
// 函数名称：void taijiaozi(void)
// 功    能：抬轿子子程序
// 入口参数：无
// 出口参数：无
//#############################################################################	 
void taijiaozi(void)
{
	uchar i;
	
	position_change[8]=10;
	position_change[9]=-10;
	position_change[10]=10;
	position_change[11]=-10;
	p_to_p(50,20);
	for(i=0;i<3;i++)
	{
		position_change[8]=-20;
		position_change[9]=20;
		position_change[10]=-20;
		position_change[11]=20;
		p_to_p(50,20);

		position_change[8]=20;
		position_change[9]=-20;
		position_change[10]=20;
		position_change[11]=-20;
		p_to_p(50,20);

	}
	position_change[8]=-10;
	position_change[9]=10;
	position_change[10]=-10;
	position_change[11]=10;
	p_to_p(50,20);

	walk(5);		//行走子程序，5步数
}