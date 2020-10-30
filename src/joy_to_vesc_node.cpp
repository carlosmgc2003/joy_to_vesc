#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64.h>


// Constante global que define la posicion en el array Float64 del eje Y del Stick Analogico Izquierdo del Joystick PS3
const int LINEAR_AXIS = 1;

class Translator {
    ros::NodeHandle n;
    ros::Subscriber joystick;
    ros::Publisher vescDriver;
    float ratio;
public:
    Translator();
    void joystickCallback(const sensor_msgs::Joy::ConstPtr&);
};

Translator::Translator(){
    n = ros::NodeHandle();
    if(n.param<float>("ratio", ratio, 0.3)){
        ROS_INFO("Coeficiente de Duty Cycle: [%.2f]", ratio);
    } else {
        ROS_INFO("Coeficiente de Duty Cycle por defecto: [%.2f]", ratio);
    }
    joystick = n.subscribe<sensor_msgs::Joy>("joy", 10, &Translator::joystickCallback, this);
    vescDriver =  n.advertise<std_msgs::Float64>("commands/motor/duty_cycle", 10);
}

void Translator::joystickCallback(const sensor_msgs::Joy::ConstPtr& joy) {
    float y_value = joy->axes[LINEAR_AXIS];
    ROS_INFO("El pad devuelve: [%.2f]", y_value);
    std_msgs::Float64 msg;
    msg.data = y_value * ratio;
    vescDriver.publish(msg);
}

int main(int argc, char **argv) {
    ros::init(argc,argv,"joy_to_vesc");
    Translator tr = Translator();
    ros::spin();
    return 0;
}