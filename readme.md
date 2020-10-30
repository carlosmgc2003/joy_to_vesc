# Joy To Vesc

## Facultad de Ingenieria del Ejercito

Paquete ROS que mapea el input de un nodo Joy_ROS a uno o mas nodos Vesc-ROS-FW-3.33, ambos requeridos. La finalidad es poder controlar duty_cycle y break del driver VESC desde un Joystick tipo PS3 para realizar pruebas de funcionamiento.

### Requisitos

* ROS Melodic
* ros-drivers/joystick_drivers
* raess1/Vesc-ROS-FW-3.33

### Instalacion

```bash
cd ~/catkin_ws/src
git clone https://github.com/carlosmgc2003/joy_to_vesc.git
cd ..
catkin_make -DCATKIN_WHITELIST_PACKAGES="joy_to_vesc"

```

### Utilizacion

```bash
rosrun joy_to_vesc joy_to_vesc_node _ratio:=0.5
```

Ratio el un multiplicador del input del joystick [-1.0;1.0]. El parametro ratio es opcional. Su valor por defecto es 0.3.
