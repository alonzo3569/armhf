#include<ros.h>
#include<motor/Motor.h> 
#include<math.h>

#define RIGHT_MOTOR 6
#define LEFT_MOTOR 3
#define RIGHT_SIGN 5
#define LEFT_SIGN 4
ros::NodeHandle nh;

void cbMotor(const motor::Motor& msg){
    int rightSp = int(abs(msg.right*255));
    int leftSp = int(abs(msg.left*255));
    analogWrite(RIGHT_MOTOR,rightSp);
    analogWrite(LEFT_MOTOR,leftSp);
    if(msg.right > 0) digitalWrite(RIGHT_SIGN,HIGH);
    else digitalWrite(RIGHT_SIGN,LOW);
    if(msg.left < 0) digitalWrite(LEFT_SIGN,HIGH);
    else digitalWrite(LEFT_SIGN,LOW);
}

ros::Subscriber<motor::Motor> sub("motor",cbMotor);

void setup(){
    nh.initNode();
    nh.subscribe(sub);

    pinMode(RIGHT_MOTOR,OUTPUT);
    pinMode(LEFT_MOTOR,OUTPUT);
    pinMode(RIGHT_SIGN,OUTPUT);
    pinMode(LEFT_SIGN,OUTPUT);

}



void loop(){
    nh.spinOnce();
}

