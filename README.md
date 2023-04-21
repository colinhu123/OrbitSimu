# Orbit Simulation

This project focuses on building a platform to be expandable to contain many factors. So far, developer have been testing gravitation force and aerodynamics forces, drag and lifting force. 

## Hi! Long time no see!

How's everything going? The developer of this project has just finished mid-term exam and he has managed to survive the difficult exam papers.



## Future Plan

1. improve visualization platform (draw the Earth) --finished(have introduced in earth)

2. improve the process of drawing -finished

2. introduce the feature AOA (angle of attacks) **finished**
 
2. introduce function of thrusting **finished**

3. design a controller to control the spacecraft to reentry atmosphere

4. improve controller to use optimization theory

5. introduce moon into theory and contian tidal force

6. turn it into three-dimension simulation

7. improve visualization process


## Develop Record

This part is created on April 21st. This is used to record problems and ideas that related to the structure of the whole project.

April 21th:
introduce the feature of the AOA. Assume the aerodynamics force does not have any moment on the spacecraft, so that it will not need to draw grid to simulate the real aerodynamics force and pressure allocation.

This time changed the whole structure into totally different way, almost all functions having been migrated to the file `structure2.py`. All functions do not have seriously tested, even run.




## This part is used to be reminder of some code and command
Some reminder of Git command
```
git add .
git commit -m message
git push

git status
```