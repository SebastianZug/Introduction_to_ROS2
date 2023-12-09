<!--
author:  André Dietrich; Sebastian Zug

mode:   Presentation

comment: Interactive LiaScript workshop at Federal University of Amazonas

-->

[![LiaScript](https://raw.githubusercontent.com/LiaScript/LiaScript/master/badges/course.svg)](https://liascript.github.io/course/?https://github.com/LiaPlayground/Brasil_2023/blob/master/README.md)

# LiaScript Tutorial

<h2> Federal University of Amazonas</h2>

__9th December 2023__

<div style="width: 55%; float: left">

| Time    | Topic                                                         |
| ------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 10:00 - 10:20 | Repetition [Implementation of Remote-Labs integrated in LiaScript](https://liascript.github.io/course/?https://raw.githubusercontent.com/LiaPlayground/Brasil_2023/main/remote_labs.md#1)         |
| 10:20 - 11:20 | [Basics of ROS]() |
| 11:20 - 11:35 | _Short break_                                                       |
| 11:35 - 12:15 | [Practical ](https://liascript.github.io/course/?https://raw.githubusercontent.com/LiaPlayground/Brasil_2023/main/best_practice.md#1)    |
| 11:00 - 12:00 | |

</div>

![partner_map](https://github.com/LiaPlayground/LiaScript-User-Symposium-2023/blob/main/pic/philosophers.png?raw=true "")<!-- style="width: 45%; float: right" -->

<div style="width: 100%;">

> All materials of this course are available on Github:
>
> https://github.com/LiaPlayground/Brasil_2023
>
> Open the tutorial part in the LiaScript Live Editor directly:
>
> [Tutorial in Live Editor](https://liascript.github.io/LiveEditor/?/show/file/https://raw.githubusercontent.com/LiaPlayground/Brasil_2023/main/basics_of_liascript.md)

</div>

## Presenters

| Name        |       eMail       |       Twitter         |
| ----------------------- |:---------------------------------------:|:-----------------------------------------------------:|
| Prof. Dr. Sebastian Zug | [sebastian.zug\@informatik.tu-freiberg.de](https://raw.githubusercontent.com/LiaPlayground/OEB-2023/main/mailto:sebastian.zug@informatik.tu-freiberg.de) |  |


+ Prof. for Software development and Robotics at Technische Universität Freiberg since 2018
+ Working with Python since 15 years especially in robotic contexts

![](images/Roboter_Bauschild.png)
![](images/Roboter_Engstelle.png)
![](images/Roboter_Engstelle2.png)
![](images/Roboter_Haustuer.png)

_Impressions of project __RobotTraces__ implemented by Hochschule Schmalkalden and Technische Universität Freiberg_

## ROS2 Installation process

> Let's run the installation process in parallel to the presentation. You can find the installation instructions in the [ROS2 documentation](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html).

```
sudo apt install ros-humble-desktop
sudo apt install ros-dev-tools
sudo apt install ros-humble-turtlesim 
```

Dont forget to source the ROS2 environment:

```
# Replace ".bash" with your shell if you're not using bash
# Possible values are: setup.bash, setup.sh, setup.zsh
source /opt/ros/humble/setup.bash
```