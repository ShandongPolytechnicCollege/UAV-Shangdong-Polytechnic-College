
<!-- markdownlint-disable -->




## Table of contents[![](https://raw.githubusercontent.com/aregtech/areg-sdk/master/docs/img/pin.svg)](#table-of-contents)
- [Team members](#Team-members)
- [Problems to solve](#Problems-to-solve)
- [Project solution](#Project-solution)
- [Advantage and innovation](#Advantage-and-innovation)
---
## Team members[![](https://raw.githubusercontent.com/aregtech/areg-sdk/master/docs/img/pin.svg)](#Usage-modes)



  - Name: Li Xiang
  - Role：Software engineer & Pilot
Responsibilities：Write code for autonomous fligt、multi-objects 
recognition system、autonomous tracking system and obstacle avoidance system，
UAV flight & maintenance。

  - Name: Yin Zhihao
  - Role：Software engineer & Team caption
Responsibilities：Computer vision algorithm & Path planning
algorithm，data analysis & visualization，machine learning。


  - Name: Cao Li
  - Role：Technical engineer & Translator
Responsibilities：Design propeller protection，test the
function of sensors，organize the team’s work on github。



---

## Problems to solve[![](https://raw.githubusercontent.com/aregtech/areg-sdk/master/docs/img/pin.svg)](#Usage-modes)


- *Project background：*
Drones can be useful in areas like large supermarkets 、museums and unmanned monitoring of warehouses where there are several threats, including: things lost, fires predicted, and thieves who are difficult to capture.

<p align="center">
    <img width="600" src="https://github.com/BRICSCN/3289457574/blob/main/12/xinjian/%E5%81%B7%E7%9B%97.gif" />
</p>

<p align="center">
    <img width="600" src="https://github.com/BRICSCN/3289457574/blob/main/12/xinjian/%E5%A4%B1%E7%81%AB.gif" />
</p>

<p align="center">
    <img width="600" src="https://github.com/BRICSCN/3289457574/blob/main/12/xinjian/%E8%BF%BD%E8%B8%AA%E6%96%B0%E9%97%BB.gif" />
</p>

<p align="center">
    <img width="900" src="https://github.com/ShandongPolytechnicCollege/UAV-Shangdong-Polytechnic-College/blob/Default/Module%20C%EF%BC%9ADebugging/Real%20Fligt/Picture/IMG_20240926_234653.jpg" />
</p>

---

## Project solution[![](https://raw.githubusercontent.com/aregtech/areg-sdk/master/docs/img/pin.svg)](#Usage-modes)


The Swarm in Blocks can be programmed either with the blocks interface or directly in Python and we developed three main launch modes, each one focused on a different application of the project, they are:

<p align="center">
    <img width="900" src="https://github.com/BRICSCN/3289457574/blob/main/12/tupian/%E8%A7%A3%E5%86%B3%E6%96%B9%E6%A1%88.png" />
</p>
<p align="center">
    <img width="900" src="https://github.com/BRICSCN/3289457574/blob/main/12/tupian/%E8%A7%A3%E5%86%B3%E6%96%B9%E6%A1%881.png" />
</p>

<p align="center">
    <img width="900" src="https://github.com/BRICSCN/3289457574/blob/main/12/tupian/%E8%A7%A3%E5%86%B3%E6%96%B9%E6%A1%882.png" />
</p>

<p align="center">
    <img width="900" src="https://github.com/BRICSCN/3289457574/blob/main/12/tupian/%E8%A7%A3%E5%86%B3%E6%96%B9%E6%A1%883.png" />
</p>

<p align="center">
    <img width="900" src="" />
</p>

Use computer vision & optical flow system to
localize the UAV.
During the manual test, release the remote
control and keep the UAV flying steadily at the
current point.

<p align="center">
    <img width="900" src="https://github.com/ShandongPolytechnicCollege/UAV-Shangdong-Polytechnic-College/blob/Default/Module%20C%EF%BC%9ADebugging/Real%20Fligt/Picture/%E8%A7%A3%E5%86%B3%E6%96%B9%E6%A1%885.png" />
</p>

<p align="center">
    <img width="900" src="https://github.com/ShandongPolytechnicCollege/UAV-Shangdong-Polytechnic-College/blob/Default/Module%20C%EF%BC%9ADebugging/Real%20Fligt/EGO%E7%AE%97%E6%B3%95.gif" />
</p>

Planning algorithm for the Ego-Planner path:
When a trajectory collides with an obstacle during
the optimisation process, a force is generated on the
trajectory depending on the collision that pushesthe
trajectory away from the obstacle.

<p align="center">
    <img width="900" src="https://github.com/ShandongPolytechnicCollege/UAV-Shangdong-Polytechnic-College/blob/Default/Module%20C%EF%BC%9ADebugging/Real%20Fligt/%E9%81%BF%E9%9A%9C%E6%A2%AF%E5%AD%90%20(2).gif" />
</p>

Visual obstacle avoidance
(VOA)


<p align="center">
    <img width="900" src="https://github.com/ShandongPolytechnicCollege/UAV-Shangdong-Polytechnic-College/blob/Default/Module%20C%EF%BC%9ADebugging/Real%20Fligt/RViz.gif" />
</p>

3D visualization tools in the ROS framework
RViz

<p align="center">
    <img width="900" src="https://github.com/BRICSCN/3289457574/blob/main/12/tupian/dand435.jpg" />
</p>


<p align="center">
    <img width="900" src="https://github.com/ShandongPolytechnicCollege/UAV-Shangdong-Polytechnic-College/blob/Default/Module%20C%EF%BC%9ADebugging/Real%20Fligt/%E8%AF%86%E5%88%AB%E7%AE%B1%E5%AD%90.gif" />
</p>

<p align="center">
    <img width="900" src="https://github.com/ShandongPolytechnicCollege/UAV-Shangdong-Polytechnic-College/blob/Default/Module%20C%EF%BC%9ADebugging/Real%20Fligt/%E5%A4%9A%E7%9B%AE%E6%A0%87.gif" />
</p>

Use machine learning to improve UAV recognition of multi-target objects

<p align="center">
    <img width="900" src="https://github.com/ShandongPolytechnicCollege/UAV-Shangdong-Polytechnic-College/blob/Default/Module%20C%EF%BC%9ADebugging/Real%20Fligt/gazebo.gif" />
</p>

Gazebo simulation：
  - （1）Multi-object recognition
  - （2）Click to lock，keep the
target in the center of the
screen



<p align="center">
    <img width="900" src="https://github.com/ShandongPolytechnicCollege/UAV-Shangdong-Polytechnic-College/blob/Default/Module%20C%EF%BC%9ADebugging/Real%20Fligt/%E8%BF%BD%E8%B8%AA%E5%B0%8F%E8%BD%A6.gif" />
</p>

Shooting Angle

<p align="center">
<img width="900" src="https://github.com/ShandongPolytechnicCollege/UAV-Shangdong-Polytechnic-College/blob/Default/Module%20C%EF%BC%9ADebugging/Real%20Fligt/%E8%BF%BD%E5%B0%8F%E8%BD%A6%E5%A4%96%E5%BD%95.gif" />
</p>

Drone View



Target tracking in the Yolov5 model can be achieved by using the prediction frame's position data and confidence
level.Use yolov5 algorithm to detect objects in the image, assign a unique ID to each object, and then perform target
object locking to track the target at a fixed distance and speed, and keep the target in the center of the screen at all times.

---

## Advantage and innovation[![](https://raw.githubusercontent.com/aregtech/areg-sdk/master/docs/img/pin.svg)](#New-Swarm-Features)


<p align="center">
    <img width="900" src="https://github.com/BRICSCN/3289457574/blob/main/12/tupian/jiewei.jpg" />
</p>

---

## Collaborative team

Aerial Tracking Drone Project Shandong Vocational and Technical College (2024) Team:

<p align="center">
    <img width="900" src="https://github.com/ShandongPolytechnicCollege/UAV-Shangdong-Polytechnic-College/blob/Default/Module%20C%EF%BC%9ADebugging/Real%20Fligt/mmexport1727382963450.jpg" />
</p>
