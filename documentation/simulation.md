# Simulation 
 
 ## 1- Robot model 
Ros give us away to create  the 3D model of a robot or its parts,  to simulate  it , by means of the URDF files.Unified Robot Description Format (URDF) is an XML format that describes a robot,its parts,..

#### Contents :

- Mostly <link> and <joint> elements
- <link>s: robot structure
- <joint>s: connections and motion constraints
### Example of URDF file :

```
<robot name="example">
	<link name="link_1"  />
   	<link name="link_2"  />
   	<join name="join_1" type="..">
     	 <parent link="link_1"  />
      	 <child link="link_2"  />
      </joint>
</robot>

```

For more information about how to create your own urdf file check this link :
http://wiki.ros.org/fr/urdf/Tutorials/Create%20your%20own%20urdf%20file

### Limitations of URDF

URDF has four limitation you should be aware of it:

##### 1.Robot descriptions cannot be changed
- Robot description read once from server.
- No standardized way to notify. 
- Risk of desynchronization. 

##### 2.Only tree structures (no loops)

-	Joint only have single parent and child
-	Only acyclic, directed graphs (or: trees) 
-	Real-world impact


##### 3.There is no sensor models ```<sensor></sensor> ```

-	Sensor meta-data can't be incorporated directly
-	Alternatives exist, but then the data is distributed
-	Diminishes value of URDF

##### 4.Low reusability of URDFs

-	Only a single <robot> tag in URDF
-	 No support for import of remote files
-	 Composite scenes have to be merged manually
-	 No way to compose multiple URDFs

##### So what is the solution ?

The solution of   these problems can be XACRO (short for XML Macros)
-	Programmatic URDF generation
-	Templates
-	Parameters XACRO
-	Import macros from other files
-	Parameterize templates
-	Composite robots & scenes easier:
-	import macro from file
-	Invoke macro
##### XACRO - Needs

-	XACRO not directly compatible with URDF
-	Transformation needed with this command:
```
$ rosrun xacro xacro /path/to/robot.xacro > robot.urdf
```
- Checks for valid XACRO file (but is not identical to check_urdf)
XACRO example

```
<xacro:macro name="arm" params="parent arm_name">
   <link name="${arm_name}_link_1"  />
   <joint name="${arm_name}_joint_1" type="..">
      <parent link="${parent}"  />
      <child link='${arm_name}_link_1"  />
   </joint>
</xacro:macro>
```

To run robot model 2019
```
roslaunch robot_description display_soja.launch
```














for more information check this link :
http://wiki.ros.org/urdf/Tutorials/Using%20Xacro%20to%20Clean%20Up%20a%20URDF%20File

## RVIZ

RVIZ  (ROS visualization) ss a tools of  3D visualization for displaying  sensor data and state information from ROS.
If you want to  visualize   current configuration on a virtual model of the robot or display sensor data  coming over ROS Topics including ultrasonic  distance measurements, sonar data, and more you need to use rviz.  


To start running rviz :
```
roscore
```
Open second terminal and write :
```  
rviz rviz
```
For example to visualize odomotry

1- clone code gihub for repository ECAM - Eurobot 2019

2- run :
```
rosrun rosserial_python serial_node.py /dev/ttyACM0
```
Make sure you use the same arduino serial

3- Run :  
```
rosrun odom odom
```
4- Run : 
```
rviz rviz
```

Visaulize the robot simulation  

Run : 
```
roslaunch robot_2019 soja.launch  viz:=true
```
## Map server

Ros offers this package to read map and offers it. but how map server works ?

He read image data then  converts color value to ternary occupancy values:
➔	 free (0)
➔	 occupied (100)
➔	 and unknown (-1)

map_server is a ROS node that reads a map from disk and offers it via a ROS service.
The current implementation of the map_server converts color values in the map image data into ternary occupancy values: free (0), occupied (100), and unknown (-1). Future versions of this tool may use the values between 0 and 100 to communicate finer gradations of occupancy.
#### Usage

map_server <map.yaml>

#### Example
```
rosrun map_server map_server mymap.yaml
```
###### To run map Eurobot 2019 :
1-
```
roscore
```
2-
```
rosrun map_server map_server maps  eurobot-map.yaml
```
3-
```
rviz rviz
```
or just one commande using launch file
```
roslaunch maps map_launch.launch
```
for more information check this link:
http://wiki.ros.org/map_server
