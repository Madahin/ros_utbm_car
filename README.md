# ros_utbm_car

## Dépendance

- Le package NMEA_node dépend de gpspipe
- Le package interface_node dépend de Qt >= 5.2 et de Marble

## Compilation

Pour compiler le projet, il suffit d'adapter et d'executer les commandes suivantes :

    cd /path/to/catkin_ws/src
    git clone https://github.com/Madahin/ros_utbm_car
    cd ..
    catkin_make
    source catkin_ws/devel/setup.bash
