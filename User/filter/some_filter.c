#include "some_filter.h"
#include "some_math.h"
#include "stdio.h"
#include "stdlib.h"
#include "math.h"



static void delay(void)
{
	;
}

int get_ad(void)
{
return 0;
}
//1、限副滤波
/*  A值可根据实际情况调整
    value为有效值，new_value为当前采样值  
    滤波程序返回有效的实际值  */
#define A 10

char value;

char filter_limit(void)
{
   char  new_value;
   new_value = get_ad();
   if ( ( new_value - value > A ) || ( value - new_value > A ))
      return value;
   return new_value;
         
}

//2、中位值滤波法
/*  N值可根据实际情况调整
    排序采用冒泡法*/
#define N  12

char filter_mid(void)
{
   char value_buf[N];
   char count,i,j,temp;
   for ( count=0;count<N;count++)
   {
      value_buf[count] = get_ad();
      delay();
   }
   for (j=0;j<N-1;j++)
   {
      for (i=0;i<N-j-1;i++)
      {
         if ( value_buf[i]>value_buf[i+1] )
         {
            temp = value_buf[i];
            value_buf[i] = value_buf[i+1]; 
             value_buf[i+1] = temp;
         }
      }
   }
	 
	 if((N&1)>0)
    temp	= 	 value_buf[(N+1)/2];
	 else 
	  temp = (value_buf[N / 2] + value_buf[N / 2 + 1]) / 2;  
	 return temp;
		 
}     

//3、算术平均滤波法


char filter_average(void)
{
   int  sum = 0;
   for ( int count=0;count<N;count++)
   {
      sum += get_ad();
      delay();
   }
   return (char)(sum/N);
}

//4、递推平均滤波法（又称滑动平均滤波法）

char value_buf[N];
char i=0;

char filter_slip(void)
{
   char count;
   int  sum=0;
   value_buf[i++] = get_ad();
   if ( i == N )   i = 0;
   for ( count=0;count<N;count++)
      sum += value_buf[count];
   return (char)(sum/N);
}


float filter_buf[255]={0};
float filter_slip_use(float get_ad,uint8_t n)
{
   static uint8_t i=0;
   uint8_t count;
    float  sum=0;
   filter_buf[i++] = get_ad;
   if ( i == n )   i = 0;
   for ( count=0;count<n;count++)
	 sum += filter_buf[count];
   
   return (float)(sum/n);
}



//5、中位值平均滤波法（又称防脉冲干扰平均滤波法）


char filter_mid_average(void)
{
  
   char value_buf[N];
   char count,i,j,temp;
	 int sum;
   for ( count=0;count<N;count++)
   {
      value_buf[count] = get_ad();
      delay();
   }
   for (j=0;j<N-1;j++)
   {
      for (i=0;i<N-j-1;i++)
      {
         if ( value_buf[i]>value_buf[i+1] )
         {
            temp = value_buf[i];
            value_buf[i] = value_buf[i+1]; 
             value_buf[i+1] = temp;
         }
      }
   }
   for(count=1;count<N-1;count++)
      sum +=value_buf [count];
   return (char)(sum/(N-2));
}

//6、限幅平均滤波法
/*
*/  
//略 参考子程序1、3

//7、一阶滞后滤波法
/* 为加快程序处理速度假定基数为100，a=0~100 */

#define a 50

char value;

char filter_later(void)
{
   char  new_value;
   new_value = get_ad();
   return (100-a)*value + a*new_value; 
}

//8、加权递推平均滤波法
/* coe数组为加权系数表，存在程序存储区。*/


char  coe[N] = {1,2,3,4,5,6,7,8,9,10,11,12};
#define sum_coe   (1+2+3+4+5+6+7+8+9+10+11+12)

char filter_push(void)
{
   char count;
   char value_buf[N];
   int  sum=0;
   for (count=0;count<N;count++)
   {
      value_buf[count] = get_ad();
      delay();
   }
   for (count=0;count<N;count++)
      sum += value_buf[count]*coe[count];
   return (char)(sum/sum_coe);
}

//9、消抖滤波法


char filter_shake_eliminate(void)
{
   char count=0;
   char new_value;
   new_value = get_ad();
   while (value !=new_value);
   {
      count++;
      if (count>=N)   return new_value;
       delay();
      new_value = get_ad();
   }
   return value;    
}

//10、限幅消抖滤波法
/*
*/
//略 参考子程序1、9 

//11、低通滤波法 
#define  a  0.7

int Low_pass(void)
{
  int input,input_last,input_take;
  input = get_ad();
//低通滤波
  input_take = a *input + (1.0f-a)*input_last;
	
	input_last = input;
	
  return input_take;
}



int Low_pass2(void)
{
  static int input,input_last,input_take;
  input = get_ad();
//低通滤波
	input_take *= a;
	
	input_take += input*(1.0f-a);

  return input_take;
}

#define T 1
int Low_pass3(void)
{
  static int input,input_take;
  input = get_ad();
//低通滤波
	input_take += 10*3.14f*T*(input-input_take);

  return input_take;
}

//11、互补滤波法 
#define k 0.9
int ang_from_acc(void)
{ 
	int acc;
	//........
	//限幅 gyro +-10;
  return acc;	
}

int ang_from_gyro(void)
{ 
	int k1=1;
	int gyro;
	gyro+=gyro*k1;
  return gyro;	
}	
	
int complementation(void)
{	
int ang_out,ang_err,ang_acc,ang_gyro;
	int k2=2;
	ang_acc=ang_from_acc();
	ang_gyro=ang_from_gyro();
	
	ang_err = ang_acc-ang_gyro;
  ang_gyro += ang_err*k2;
	
return ang_out;
}

//13.卡尔曼滤波法
double frand()
{
	return 2*((rand()/(double)RAND_MAX) - 0.5); //随机噪声
}

int kalman_filte_test(float z_measure,float Q,float R)
{

float x_last=0;     //系统初始值
float p_last=0.02;  //信任权重
//float Q=0.018;
//float R=0.542;
float kg;     //kg 为 kalman filter
float x_mid;  //估计值 （预测，假定和之前一样）
float x_now;  //估计出最优值（output）
float p_mid;       
float p_now;
//float z_real=0.56;//0.56
//float sumerror_kalman=0;
//float sumerror_measure=0;
int i;
//x_last=z_real+frand()*0.03;
//x_last=z_real;
x_mid=x_last;
	
for(i=0;i<20;i++)
{
		x_mid=x_last;                    //x_last=x(k-1|k-1),x_mid=x(k|k-1)
		p_mid=p_last+Q;                 //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=噪声
		kg=p_mid/(p_mid+R);          		//kg 为 kalman filter，R 为噪声
//		z_measure=z_real+frand()*0.03;	//测量值
		x_now=x_mid+kg*(z_measure-x_mid);		//估计出的最优值
		p_now=(1-kg)*p_mid;									//最优值对应的 covariance
	
//		printf("Real position: %6.3f \n",z_real); 		//显示真值
//		printf("Mesaured position: %6.3f [diff:%.3f]\n",z_measure,fabs(z_real-z_measure)); //显示测量值以及真值与测量值之间的误差																																						
//		printf("Kalman position: %6.3f [diff:%.3f]\n",x_now,fabs(z_real - x_now)); //显示	kalman 估计值以及真值和卡尔曼估计值的误差
	
//		sumerror_kalman += fabs(z_real - x_now); 					//kalman 估计值的累积误差
//		sumerror_measure += fabs(z_real-z_measure); //真值与测量值的累积误差
		p_last = p_now; 														//更新 covariance 值
		x_last = x_now; 																//更新系统状态值
}

//printf("总体测量误差 : %f\n",sumerror_measure); //输出测量累积误差
//printf("总体卡尔曼滤波误差: %f\n",sumerror_kalman); //输出 kalman 累积误差
//printf("卡尔曼误差所占比例: %d%% \n",100-(int)((sumerror_kalman/sumerror_measure)*100));

return  x_now;
	
}

int kalman_filte(float z_measure,float Q,float R,float x_last,float p_last)
{

//float x_last=0;     //系统初始值
//float p_last=0.02;  //信任权重
//float Q=0.018;
//float R=0.542
float kg;     //kg 为 kalman filter
float x_mid;  //估计值 （预测，假定和之前一样）
float x_now;  //估计出最优值（output）
float p_mid;       
float p_now;

int i;

x_mid=x_last;
	
for(i=0;i<20;i++)
{
		x_mid=x_last;                      		//x_last=x(k-1|k-1),x_mid=x(k|k-1)
		p_mid=p_last+Q;                    	    //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=噪声
		kg=p_mid/(p_mid+R);          			//kg 为 kalman filter，R 为噪声

		x_now=x_mid+kg*(z_measure-x_mid);		//估计出的最优值
		p_now=(1-kg)*p_mid;						//最优值对应的 covariance

		p_last = p_now; 						//更新 covariance 值
		x_last = x_now; 						//更新系统状态值
}


return  x_now;
	
}


float angle, angle_dot; 	
float Q_angle=0.001;// 过程噪声的协方差
float Q_gyro=0.003;//0.003 过程噪声的协方差 过程噪声的协方差为一个一行两列矩阵
float R_angle=0.5;// 测量噪声的协方差 既测量偏差
float dt=0.005;//                 
char  C_0 = 1;
float Q_bias, Angle_err;
float PCt_0, PCt_1, E;
float K_0, K_1, t_0, t_1;
float Pdot[4] ={0,0,0,0};
float PP[2][2] = { { 1, 0 },{ 0, 1 } };

void Kalman_Filter(float Accel,float Gyro)		
{
	angle+=(Gyro - Q_bias) * dt; //先验估计
	Pdot[0]=Q_angle - PP[0][1] - PP[1][0]; // Pk-先验估计误差协方差的微分

	Pdot[1]=-PP[1][1];
	Pdot[2]=-PP[1][1];
	Pdot[3]=Q_gyro;
	PP[0][0] += Pdot[0] * dt;   // Pk-先验估计误差协方差微分的积分
	PP[0][1] += Pdot[1] * dt;   // =先验估计误差协方差
	PP[1][0] += Pdot[2] * dt;
	PP[1][1] += Pdot[3] * dt;
		
	Angle_err = Accel - angle;	//zk-先验估计
	
	PCt_0 = C_0 * PP[0][0];
	PCt_1 = C_0 * PP[1][0];
	
	E = R_angle + C_0 * PCt_0;
	
	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;
	
	t_0 = PCt_0;
	t_1 = C_0 * PP[0][1];

	PP[0][0] -= K_0 * t_0;		 //后验估计误差协方差
	PP[0][1] -= K_0 * t_1;
	PP[1][0] -= K_1 * t_0;
	PP[1][1] -= K_1 * t_1;
		
	angle	+= K_0 * Angle_err;	 //后验估计
	Q_bias	+= K_1 * Angle_err;	 //后验估计
	angle_dot   = Gyro - Q_bias;	 //输出值(后验估计)的微分=角速度
//	rt_kprintf("angle %d\r\n",(int)angle_dot);
}

#define lvbo_K1 0.2
float Yijielvbo(float angle_m, float gyro_m)
{
	float angle;
	angle = lvbo_K1 * angle_m+ (1-lvbo_K1) * (angle + gyro_m * 0.005);

//	rt_kprintf("angle %d\r\n",(int)angle);
	return angle;
}
