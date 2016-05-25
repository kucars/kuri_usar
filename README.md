# kuri_usar
Urban Seacrh &amp; Rescue (USAR) 



USAR related tasks implementation

In order run the simulation environment the following Packages are needed: 

- ROS
- RotorS rotor_simulator    
- mavros     
- Firmware     
- kuri_usar_sim simulation environment    

# Installing using rosinstall
```
cd <catkin_ws>
$ wstool init src
$ wstool set -t src kuri_usar https://github.com/kuri-kustar/kuri_usar.git --git
$ wstool merge -t src https://github.com/kuri-kustar/kuri_usar/blob/master/kuri_usar.rosinstall
$ wstool update -t src
$ rosdep install -y -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO
$ catkin_make
```
