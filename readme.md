# LadyBug

LadyBug is a bug algorithm based on Received Signal Strength Indication (RSSI). It has two distinct parts. The first part is localization, where the position of the beacon in the robot-centric coordinates system are calculated. The second part is navigation. where the robot navigates to the beacon while avoiding obstacles found on it's way. 

LadyBug was based on the key idea of I-Bug [[1]](#1). In  order  to  evaluate  the  performance  of  the LadyBug algorithm  and  provide  an  empirical  comparison  against  the I-Bug algorithm, we implemented both algorithms.

## Usage

In order to use the provided code, Webots is needed [[2]](#2). You can download webots from https://cyberbotics.com/ A demo world is provided (```plain.wbt```) as well as the controller implementing both algorithms (```bug.c```). 


## Citation
A. Lentzas and D. Vrakas, "LadyBug. An Intensity based Localization Bug Algorithm," 2020 25th IEEE International Conference on Emerging Technologies and Factory Automation (ETFA), Vienna, Austria, 2020, pp. 682-689, doi: 10.1109/ETFA46521.2020.9212115.

## References
[1] K. Taylor and S. M. LaValle, “I-bug: An intensity-based bug algorithm,”in2009 IEEE International Conference on Robotics and Automation,May 2009, pp. 3981–3986.

[2] O.  Michel,  “Webots:  Professional  mobile  robot  simulation,”Journalof Advanced Robotics Systems,   vol.   1,   no.   1,   pp.   39–42,   2004.[Online].   Available:   http://www.ars-journal.com/International-Journal-of- Advanced-Robotic-Systems/Volume-1/39-42.pdf
