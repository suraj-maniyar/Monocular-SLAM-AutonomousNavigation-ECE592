
#include <ros/ros.h>

#include <std_msgs/String.h>
#include <geometry_msgs/PoseArray.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <math.h>
#include <whycon_track/JHPWMPCA9685.h>
#include <sstream>
#define PI 3.14159265
int servoMin = 100 ;
int servoMax = 530 ;
PCA9685 *pca9685 = new PCA9685();
int ini = 100;
float cst = 0.01;
int m = 1;
double val,y,z;
int fina;
int curr = 350;
void trackerCallback(const geometry_msgs::PoseArray& data)
{
	//int i;
//for(i=0;i<3;i++)
  
	
	y = data.poses[0].position.y;
	z = data.poses[0].position.z;
	//int c = 1;
	ini = ini + (m*10);
	val = atan(y/z) * 180 / PI;
	fina = 2 * (int) val;
	//float c = y - 0.01
	if(curr+fina < servoMax && curr+fina > servoMin)
		curr = curr + fina;
ROS_INFO("Y: %f,- Z:%f and curr is %d and val is %f fin is %d",y,z,curr,val,fina);
//	printf("INI is %d",ini);
if(pca9685 != NULL)	
{	
	pca9685->setPWM(1,0,curr);
	sleep(0.2) ;
}
else
	ROS_INFO("It is NULL");
	

	if(ini>servoMax)
		m=m * -1;
	if(ini<servoMin)
		m = m * -1;
}


int main(int argc, char **argv)
{

//PCA9685 *pca9685 = new PCA9685() ;
    int err = pca9685->openPCA9685();
    if (err < 0){
        printf("Error: %d", pca9685->error);
    } else {

	printf("PCA9685 Device Address: 0x%02X\n",pca9685->kI2CAddress) ;
        pca9685->setAllPWM(0,0) ;
        pca9685->reset() ;
        pca9685->setPWMFrequency(60) ;
	pca9685->setPWM(1,0,300);
	sleep(0.2) ;
pca9685->setPWM(1,0,300);
	sleep(0.2) ;
pca9685->setPWM(1,0,280);
	sleep(0.2) ;
pca9685->setPWM(1,0,260);
	sleep(0.2) ;
pca9685->setPWM(1,0,240);
	sleep(0.2) ;
	int j = 0;
	while(j < 0)
{
	for(int i=servoMin; i<=servoMax; i++){
			pca9685->setPWM(1,0,i);
			sleep(0.02) ;
		}
	sleep(2);
		for(int i=servoMax; i>=servoMin; i--){
			pca9685->setPWM(1,0,i);
			sleep(0.02) ;
		}
	
	sleep(2);
j++;
}
	ros::init(argc, argv, "whycon_track");
  //ros::NodeHandle n;
	  ros::NodeHandle ns;
	  ros::Subscriber sub = ns.subscribe("/whycon/poses", 10, trackerCallback);
  //ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
	  ros::spin();
        
        // 27 is the ESC key
       /* printf("Hit ESC key to exit\n");
        while(pca9685->error >= 0 && getkey() != 27){
		for(int i=servoMin; i<=servoMax; i++){
			pca9685->setPWM(0,0,i);
			sleep(0.02) ;
		}
		for(int i=servoMax; i>=servoMin; i--){
			pca9685->setPWM(0,0,i);
			sleep(0.02) ;
		}
            //pca9685->setPWM(0,0,servoMax) ;
            sleep(2) ;
	
        }
        sleep(1);*/
    }
    pca9685->closePCA9685();
  

  return 0;
}
