//�����ӳ���
#include <reg60s2.h>
#include <delay.h>
#include <wifi-module.h>
#include <shr-51-c.h>
#include <math.h> 
#include <adc.h>
#include <walking.h>

uchar  position[24]={0};             //���ڼ�¼24�������λ��
/*
        �����Ӧ�˿�˵����
         position[0]       ����������>>      P0.0          x1���  x��            
         position[1]       ����������>>      P0.1          x2���                 
         position[2]       ����������>>      P0.2          x3���                 
         position[3]       ����������>>      P0.3          x4���                 
         position[4]       ����������>>      P0.4          x5���                 
         position[5]       ����������>>      P0.5          x6���                 
         position[6]       ����������>>      P0.6          x7���                 
         position[7]       ����������>>      P0.7          x8���
		                  
         position[8]       ����������>>      P1.0          y1���  y��        
         position[9]       ����������>>      P1.1          y2���               
         position[10]      ����������>>      P1.2          y3���                 
         position[11]      ����������>>      P1.3          y4���                 
         position[12]      ����������>>      P1.4          y5���                 
         position[13]      ����������>>      P1.5          y6���                 
         position[14]      ����������>>      P1.6          y7���                 
         position[15]      ����������>>      P1.7          y8���    
		              
         position[16]      ����������>>      P2.0	       z1���  z��
         position[17]      ����������>>      P2.1	       z2���
         position[18]      ����������>>      P2.2		   z3���
         position[19]      ����������>>      P2.3		   z4���
         position[20]      ����������>>      P2.4		   z5���
         position[21]      ����������>>      P2.5		   z6���
         position[22]      ����������>>      P2.6		   z7���
         position[23]      ����������>>      P2.7		   z8���
*/
uchar position_initial[24];
int position_change[24]; 
sint16 jichu[24];

uchar pick_up[8];           //kouchu[8];����io�������������������λ
uchar arr[8];               //�ṩ����ռ� paixu_ncha[8]=0;      //�ṩ����ռ�

uchar t0bit=0;			    //��ʱ��0������ͬ��־λ
uchar total;			    //ǧ�ֹ����ܹ���̨��M
uchar part;				    //��������̨��N
uchar change;			    //ÿ�������ı仯��S

//#############################################################################
// �������ƣ�low_level_500u(uchar time)
// ����˵����PWM�źŵ͵�ƽʱ���ӳ��򣬿��ƶ��PWM�źŵĵ͵�ƽʱ��������ת�����ٶ�
// ��ڲ������͵�ƽʱ�䣬500uΪ����
// ���ڲ�������
//#############################################################################
void low_level_500u(sint16 time)
{
  sint16 i; 
  for(i=0;i<time;i++)
  {delay500us(1);}
}
//#############################################################################
// �������ƣ�void t0_init(void)
// ����˵����t0��ʱ����ʼ�������ڶ��2.5ms��ʱ
// ��ڲ�������
// ���ڲ�������
//#############################################################################
void t0_init(void)
{
  TMOD  = TMOD & 0xf0;        //��ʼ����ʱ��1�ļ�����ʽΪ��ʽ1
  TMOD  = TMOD | 0x01;

  TH0=0xf7;			          //22.1184MHz,2.5Ms��ʱdc00
  TL0=0x00;
  ET0=1;						
  EA=1;
  t0bit=1;
}
//#############################################################################
// �������ƣ�low_level_t0(uchar TH,uchar TL)
// ����˵����ͬ���ڶ�ʱ��0���ü���������ʹÿ�仯һ���仯����������ͬ
// ��ڲ�����THTL��ʱ����ֵ��
// ���ڲ�������
//#############################################################################
 void low_level_t0(uint THTL)
{
  TH0=THTL>>8;			  //22.1184MHz,2.5Ms��ʱdc00
  TL0=THTL;
  t0bit=0;
  TR0=1;
}
//#############################################################################
// �������ƣ�void T0_Interrupt(void) interrupt 1
// ����˵����PWM�źŵ͵�ƽʱ���ӳ��򣬿��ƶ��PWM�źŵĵ͵�ƽʱ��������ת�����ٶ�
// ��ڲ������͵�ƽʱ�䣬500uΪ����
// ���ڲ�������
//#############################################################################	 
void T0_Interrupt(void) interrupt 1
{  TH0=0xdc;		     //22.1184MHz,2.5Ms��ʱdc00
   TL0=0x00;
   t0bit=1;
}
//#############################################################################
// �������ƣ�void ForwardDelay()
// ����˵����ǧ�ֹ�����ͬ��ǰ��ʱ�ӳ���
// ��ڲ�������
// ���ڲ�������
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
// �������ƣ�void BackDelay()
// ����˵����ǧ�ֲ�ͬ������ʱ�ӳ���
// ��ڲ�������
// ���ڲ�������
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
// �������ƣ�void array( )
// ����˵���������ӳ��򣬽������ڵ�8λ����ʱ��ĳ�������
// ��ڲ�������
// ���ڲ�������
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
	//����
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
// �������ƣ�int max(uchar total��int M)
// ����˵�����������о���ֵ����Ԫ�ص��ӳ���
// ��ڲ�������
// ���ڲ�������
//#############################################################################
int max(uchar total,int M)
{ uchar i;
	//�����
	for(i=0;i<total;i++)
	{   if (abs(position_change[i])>M)
	        M=abs(position_change[i]);
	}	
		return M;			 
}
//#############################################################################
//�������ƣ�void p_to_p(int delay)
//����˵����position �仯�ӳ���ʵ��ͬ��ͬ��
//��ڲ�����delay=��ʱ��define:p-to-p����ѭ���������Լ���������ͨ����������������
//���ڲ�������
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
   if(define==0)					 //����ѭ������M
   {
     M=max(24,M);
	 if(M>20)
     M=20;
   }   
   else
   	{M=define; 	 					 //ѭ�������������
	}
	 for(i=1;i<=M;i++)				 //��ʼѭ��M�����
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
   for(i=0;i<24;i++) 		  		 //��󲹳�ÿ����������  		 
   {
      position[i]=position_buffer[i]+position_change[i];
      position_change[i]=0;
   }
  PWM_24();
  low_level_500u(30);   
}

//##############################################################################
//�������ƣ�void relative(int a1,int a2,int a3,int a4,int a5,int a6)
//����˵��������24������ı仯��
//��ڲ�������
//���ڲ�������
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
  i=a2;i=a3;i=a4;i=a5;i=a6;i=a7;i=a8;i=a9;i=a10;i=a11;i=a12;i=a13;i=a14;i=a15;i=a16;i=a17;			  //����������
}

//#############################################################################
// �������ƣ�void initial_position(void)
// ����˵������ʼλ���ӳ��򣬸��ݸ�������Ĳ�ͬλ�����ó�ʼλ�á�
// ��ڲ�������
// ���ڲ�������
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
             //PB��
             position[8]=122;      //95
             position[9]=116;      //111
             position[10]=129;     //131
             position[11]=120;     //113
             position[12]=218;      //206
             position[13]=114;      //114
             position[14]=34;       //30
             position[15]=119;			 	//19
			 //PC��
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
// �������ƣ�void initial_position(void)
// ����˵������ʼλ���ӳ��򣬸��ݸ�������Ĳ�ͬλ�����ó�ʼλ�á�
// ��ڲ�������
// ���ڲ�������
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
//�������ƣ�void sit_down(uchar step)
//����˵���������ӳ���ʹ�����˴���΢��״̬���������ߡ�
//��ڲ�����setp, ��ʾ�¶׵ĳ̶ȡ�
//���ڲ�������
//#############################################################################
void sit_down(uchar step)
{        
         uchar i;
	 
         for(i=0;i<step;i++)
		    {
			   position[0]--;              //�����¶�
			   position[1]+=2;             
			   position[2]++;               
			   
               position[3]++;              //�����¶�
			   position[4]-=2;              
			   position[5]--;               
			   PWM_24();
               low_level_500u(30);
			}			
}
//#############################################################################
//�������ƣ� void stand_up(uchar setp)
//����˵���������ӳ�����Щ��������վֱ���������
//��ڲ�����setp, ��ʾ�����ĳ̶ȡ�
//���ڲ�������
//#############################################################################
void stand_up(uchar setp)
{        
         uchar i;
		
         for(i=0;i<setp;i++)
		    {
			    position[0]++;             //��������
			    position[1]-=2;            
			    position[2]--;             

                position[3]--;             //��������
			    position[4]+=2;           
			    position[5]++;            
			    PWM_24();
                low_level_500u(20);       		
			}
}	
//#############################################################################
//�������ƣ� void turn_left(uchar step)
//����˵������ת�ӳ���
//��ڲ�����step, ��ʾ��ת������
//���ڲ�������
//#############################################################################

void turn_left(uchar step)
{   uchar j,i;  
               sit_down(50);
		       delay500ms(1);
			  
		  for(i=0;i<step;i++)                     //�����                           
			{	   
			  for(j=0;j<20;j++)                     //�����                           
			    {
			     position[8]-=1;
				 position[9]+=1;
				 position[10]-=1;
				 position[11]+=1;
				 PWM_24();
                 low_level_500u(5);								
                }
			  for(j=0;j<20;j++)                    //̧����                     
			    {
			     position[3]+=1;
				 position[4]-=2;
				 position[5]-=1;
				   PWM_24();
                 low_level_500u(5);								
                }	
			  for(j=0;j<20;j++)                    //������               
			    {
			     position[0]+=1;
				 position[2]+=1;
				 position[3]+=1;
				 position[5]+=1;
				   PWM_24();
                 low_level_500u(5);								
                }
			  for(j=0;j<20;j++)                     //������                     
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
//�������ƣ�void turn_left(uchar step)
//����˵������ת�ӳ���
//��ڲ�����step, ��ʾ��ת������
//���ڲ�������
//#############################################################################
 void turn_right(uchar step)
{       uchar j,i;    
           sit_down(50);
		     delay500ms(1);
		for(i=0;i<step;i++)                                              
		   {
			 for(j=0;j<20;j++)                    //�Ҳ���                         
			    {
			     position[8]+=1;
				 position[9]-=1;
				 position[10]+=1;
				 position[11]-=1;
				 PWM_24();
                 low_level_500u(5);								
                }
			  for(j=0;j<20;j++)                    //̧����                     
			    {
			     position[0]-=1;
				 position[1]+=2;
				 position[2]+=1;
				   PWM_24();
                 low_level_500u(5);								
                }
			  for(j=0;j<20;j++)                    //������               
			    {
			     position[0]-=1;
				 position[2]-=1;
				 position[3]-=1;
				 position[5]-=1;
				   PWM_24();
                 low_level_500u(5);								
                }
			  for(j=0;j<20;j++)                     //������                     
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
//�������ƣ� void l_pyi(uchar step)
//����˵������ƽ���ӳ���
//��ڲ�����step, ��ʾ��ƽ�ƴ�����
//���ڲ�������
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
//�������ƣ� void r_pyi(uchar step)
//����˵������ƽ���ӳ���
//��ڲ�����step, ��ʾ��ƽ�ƴ�����
//���ڲ�������
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
//�������ƣ� void fwc_paxia()
//����˵�������Գ�ſ�£����Գ�֮ǰ��ſ�¶�������������������
//��ڲ�������
//���ڲ�������
//#############################################################################
void fwc_paxia(void)
{
      uchar i;
   
	      for(i=0;i<40;i++)
		     {
			   position[0]-=3;              //  ��������
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
			  position[2]++;              //  ��ǰ��
			  position[5]--;               
			  position[6]-=2;             
              position[7]+=2;             
			  PWM_24();
                    low_level_500u(10);
		     }
			 
			  for(i=0;i<10;i++)
		    {
			  position[2]++;              //  ��ǰ��
			  position[5]--;             
			    PWM_24();
                    low_level_500u(10);
		     }
	  	for(i=0;i<80;i++)
		    {
			   position[0]+=2;             
			   position[3]-=2;               
			   position[6]++;               //�����
    	       position[7]--;                
		      PWM_24();
                    low_level_500u(10);
		     }
}									  
//#############################################################################
//�������ƣ� void fwc_qilai()
//����˵�������Գ����������Գ�����������������������
//��ڲ�������
//���ڲ�������
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
               position[6]-=1;              // ����ǰ��
    	       position[7]+=1;             
		       PWM_24();
               low_level_500u(10);
		     }
		for(i=0;i<30;i++)
		    {
			  position[2]--;                //  �����
			  position[5]++;                
			  PWM_24();
                    low_level_500u(45);
		     }
	 for(i=0;i<60;i++)
		    {
			  position[2]--;               //  �����
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
			   position[0]+=2;             // ����������
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
		
		       initial_position();               //���û����˳�ʼλ���ӳ���
			 delay10ms(100);                        
		
}

//#############################################################################
//�������ƣ� void fwc(uchar step)
//����˵�������Գ��ӳ���
//��ڲ�����step, ��ʾ���ԳŴ�����
//���ڲ�������
//#############################################################################
void fwc(uchar step)
{
      uchar i,j;
	  
		       initial_position();               //���û����˳�ʼλ���ӳ���
         fwc_paxia();
		
            for(i=0;i<30;i++)
		      {
			   position[6]++;              
    	       position[7]--;             
		      PWM_24();
                    low_level_500u(10);
		       }
             for(j=0;j<step;j++)           //���Գ�
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
//�������ƣ� void goal_l(uchar step)
//����˵�����������(վ��)�ӳ���
//��ڲ�����step, ��ʾ���������
//���ڲ�������
//#############################################################################
void goal_l(uchar step)
{ uchar i;
       sit_down(44);
		 delay500ms(1);
	  while(step)
       {  
	    
	     step--;
         for(i=0;i<8;i++)            //˫�ַֿ�
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
			
			for(i=0;i<16;i++)       //̧����
               {
			        position[0]--;  
					position[1]+=2;  
                  	position[2]++;  
					PWM_24();
                    low_level_500u(10);
			   }
 
            for(i=0;i<16;i++)      //����//
               {			      
					position[0]++;  
                  	position[3]++;  
					PWM_24();
                    low_level_500u(10);
			   }

		                           //����
            position[0]=position[0]-24;  
           	position[3]=position[3]-24;  
            PWM_24();

			for(i=0;i<1;i++)      //���ȼ���
               {
                  	position[0]=position[0]-38;  
					PWM_24();
                    low_level_500u(10);
			   }

			for(i=0;i<19;i++)      //�����ջ�
               {
                  	position[0]+=2; 
					PWM_24();
                    low_level_500u(10);
			   }

			for(i=0;i<8;i++)      //����
               {
                  	position[0]++;  
					position[3]++;  
					PWM_24();
                    low_level_500u(10);
			   }

		    for(i=0;i<16;i++)      //������
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
            for(i=0;i<8;i++)         //˫�ֺ�£//
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
//�������ƣ� void goal_r(uchar step)
//����˵�����ҽ�����(վ��)�ӳ���
//��ڲ�����step, ��ʾ���������
//���ڲ�������
//#############################################################################
void goal_r(uchar step)
 {
		uchar i=0;
		sit_down(44);
		delay500ms(1);
        while(step) 
 	      {
		    step--;
			for(i=0;i<8;i++)         //˫�ַֿ�
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
          		
			for(i=0;i<16;i++)        //̧����
               {
			        position[3]++; 
					position[4]-=2;  
                  	position[5]--;  
					PWM_24();
                    low_level_500u(10);
			   }

            for(i=0;i<16;i++)       //����
               {
			      
					position[0]--;  
                  	position[3]--;  
					PWM_24();
                    low_level_500u(10);
			   }

			   //����
            position[0]=position[0]+24;   
           	position[3]=position[3]+24;  
            PWM_24();

			for(i=0;i<1;i++)       //���ȼ���
               {
                  	position[3]=position[3]+38;  
					PWM_24();
                    low_level_500u(10);
			   }

			for(i=0;i<19;i++)      //�����ջ�
               {
                  	position[3]--; 
					position[3]--;  
					PWM_24();
                    low_level_500u(10);
			   }

			for(i=0;i<8;i++)      //����
               {
                  	position[0]--;  
					position[3]--;  
					PWM_24();
                    low_level_500u(10);
			   }

		    for(i=0;i<16;i++)      //������//
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
             for(i=0;i<8;i++)         //˫�ֺ�£//
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
//�������ƣ� void qianpaxia(void)
//����˵����ǰſ���ӳ���
//��ڲ�������
//���ڲ�������
//#############################################################################
void qianpaxia(void)
{
           uchar i;
               fwc_paxia();
		     for(i=0;i<65;i++)
		       {
			    position[6]+=2;              // �����
    	        position[7]-=2;              
				PWM_24();
                low_level_500u(30);
		       }
			     delay1s(1);
				 delay1s(1);
}
//#############################################################################
//�������ƣ� void qianpq(void)
//����˵����ǰ�����ӳ���
//��ڲ�������
//���ڲ�������
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
//�������ƣ� void hp(void)
//����˵������ſ���ӳ���
//��ڲ�������
//���ڲ�������
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
//�������ƣ�void hp(void)
//����˵�����������ӳ���
//��ڲ�������
//���ڲ�������
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
//�������ƣ� void yangwoqizuo(uchar step)
//����˵�������������ӳ���
//��ڲ�����step, ��ʾ���������Ĵ�����
//���ڲ�������
//#############################################################################

void yangwoqizuo(uchar step)
{
         uchar i,j;
            hp();
			delay1s(1);
	      for(i=0;i<5;i++)
		    {
			 position[8]--;              //  ˫�Ⱥ�£ ��������
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
		     initial_position();               //���û����˳�ʼλ���ӳ���
			 hpq();                     
}

//#############################################################################
//�������ƣ�void jingli_l(void)
//����˵�����������֣��ӳ���
//��ڲ�������
//���ڲ�������
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
//�������ƣ� void jingli_r(void)
//����˵�����������֣��ӳ���
//��ڲ�������
//���ڲ�������
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
//�������ƣ� void qgf(void)
//����˵����ǰ�����ӳ���
//��ڲ�����step, ��ʾǰ����������
//���ڲ�������
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
//�������ƣ� void hgf(void)
//����˵����������ӳ���
//��ڲ�����step, ��ʾ�����������
//���ڲ�������
//#############################################################################
void hgf(void)
{
              uchar i;
			
	  initial_position();                  //���û����˳�ʼλ���趨��1���ӳ���
		  	for(i=0;i<5;i++)
		      {
			   position[8]--;              //   ˫�Ⱥ�£ ��������
			   position[9]++;                
			   position[10]++;             
			   position[11]--;               
			   PWM_24();
    low_level_500u(10);
		      }
	        for(i=0;i<20;i++)
		      {
			   position[0]+=5;              //  ��������
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
			  position[2]--;               // �����
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
// �������ƣ�void daoli(void)
// ��    �ܣ���������
// ��ڲ�������
// ���ڲ�������
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
	    
 initial_position();               //���û����˳�ʼλ���ӳ���
}                                           	
//#############################################################################
// �������ƣ�void pch(void)
// ��    �ܣ������溯��
// ��ڲ�������
// ���ڲ�������
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
	   initial_position();               //���û����˳�ʼλ���ӳ���
}
//#############################################################################
// �������ƣ�void dt(void)
// ��    �ܣ�����ֱ������
// ��ڲ�������
// ���ڲ�������
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
 initial_position();               //���û����˳�ʼλ���ӳ���
}
//#############################################################################
// �������ƣ�void taijiaozi(void)
// ��    �ܣ�̧�����ӳ���
// ��ڲ�������
// ���ڲ�������
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

	walk(5);		//�����ӳ���5����
}