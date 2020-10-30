#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64.h>


// Constante global que define la posicion en el array Float64 del eje Y del Stick Analogico Izquierdo del Joystick PS3
const int LINEAR_AXIS = 1;
const int RED_BUTTON = 1;
const double DEFAULT_RATIO = 0.3;


// Clase que convierte el input del Joystick en instrucciones para Drivers VESC
class Translator {
    ros::NodeHandle n;
    ros::NodeHandle private_n;
    ros::Subscriber joystick;
    ros::Publisher vescMotor;
    ros::Publisher vescBrake;
    double ratio;
public:
    Translator();
    void joystickCallback(const sensor_msgs::Joy::ConstPtr&);
    void setRatio();
};

Translator::Translator(){
    // Manejador del Nodo
    n = ros::NodeHandle();
    // Instancia privada solamente para aceder a los parametros
    private_n = ros::NodeHandle("~");
    setRatio();
    // Captura del Parametro ratio, que define el multiplicador de reduccion del motor
    
    joystick = n.subscribe<sensor_msgs::Joy>("joy", 10, &Translator::joystickCallback, this);
    vescMotor =  n.advertise<std_msgs::Float64>("commands/motor/duty_cycle", 10);
    vescBrake =  n.advertise<std_msgs::Float64>("commands/motor/brake", 10);
}

void Translator::setRatio() {
    if(private_n.param<double>("ratio", ratio, DEFAULT_RATIO)){
        if(ratio >= 1.0 || ratio <= 0.1) {
            ratio = DEFAULT_RATIO;
            ROS_INFO("El ratio seteado es invalido.");
        }
        ROS_INFO("Ratio seteado [%.2f]", ratio);
    } else {
        ROS_INFO("Ratio por defecto [%.2f]", ratio);
    }
}

void Translator::joystickCallback(const sensor_msgs::Joy::ConstPtr& joy) {
    // Captura de los valores de los botones de Joystick
    const double y_value = joy->axes[LINEAR_AXIS];
    const int red_value = joy->buttons[RED_BUTTON];
    // Instancia de los mensajes
    std_msgs::Float64 msgMotor;
    std_msgs::Float64 msgBrake;

    // Carga de los datos
    msgMotor.data = y_value * ratio;
    msgBrake.data = red_value;

    // Feedback en el log
    ROS_INFO("El pad devuelve: [%.2f], VESC recibe [%.2f], Brake en [%.2f]", y_value, msgMotor.data, msgBrake.data);
    
    // Publicacion de los datos
    vescMotor.publish(msgMotor);
    // Solo se publica freno cuando cambia
    if(red_value != 0)
        vescBrake.publish(msgBrake);
}

int main(int argc, char **argv) {
    ros::init(argc,argv,"joy_to_vesc");
    Translator tr = Translator();
    ros::spin();
    return 0;
}