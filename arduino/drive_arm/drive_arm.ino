#include <ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define AMOUNT_MOTORS 4
#define SERVO_FREQ 60 // Analog servos run at ~50 Hz updates

int MIN_IMP [AMOUNT_MOTORS] ={500, 500, 500, 500};
int MAX_IMP [AMOUNT_MOTORS] ={2500, 2500, 2500, 2500};
int MIN_ANG [AMOUNT_MOTORS] ={0, 0, 0, 0};
int MAX_ANG [AMOUNT_MOTORS] ={180, 180, 180, 180};

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver();

int imp = 0;

ros::NodeHandle nh;

void drive_motor(const std_msgs::Float64MultiArray& joint_sub) {
    for (size_t n = 0; n < AMOUNT_MOTORS; n++) {
        imp = map(joint_sub.data[n], MIN_ANG[n], MAX_ANG[n], MIN_IMP[n], MAX_IMP[n]);
        pca.setPWM(n, 0, imp);
    }
}

ros::Subscriber<std_msgs::Float64MultiArray> sub("/joint_positions", drive_motor);

void setup()
{
    nh.getHardware()->setBaud(57600);
    nh.initNode();
    nh.subscribe(sub);

    pca.begin();
    pca.setPWMFreq(SERVO_FREQ);
}

void loop()
{
    nh.spinOnce();
    delay(1);
}
