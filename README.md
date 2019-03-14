# DE_robotics1
Design Engineering Robotics 1 module coursework

Created by: Anusha Sonthalia

Contributors: Oliver Hoare, Sophie Owen, Francesca Suer, Laerke Mop Rasmussen, Yeonjin Kim

Date Created: Sunday 24th Feb 2019

Date Last Edited: Thursday 14th March 2019


## Description
This repository is a collection of all the code our team wrote in order to build a brick structure using the Imperial College London version of the [Baxter Research Robot](http://sdk.rethinkrobotics.com/wiki/Home), DENIRO. All scripts are written in python and all code must be run in a [catkin workspace](http://wiki.ros.org/catkin/workspaces). The details of our project brief, instructions on running the submission code, and the expected results can all be found in the videos folder.

## Organisation
Code_Snippets folder contains all parts of code written by team members in the project.

Final_Submission contains code to be submitted for assessment.

Models contains the modified URDF files which we used in order to spawn objects into the simulation.

Examples contains the original example code given to us along with the DENIRO virtual machine (identical to the example code given with the original Baxter virtual machine).

Videos contains four files. 
* Detials the project brief
* Code running on the physcial robot
* Code running in the Gazebo simulation 
* How to run the simulation code

## Installation
We used VMware Workstation in order to run our virtual machine. This is a paid product which we were provided with through our university. We also had a customised virtual machine to run with DENIRO however this code can be run if the Baxter vritual machine [setup instructions](http://sdk.rethinkrobotics.com/wiki/Hello_Baxter#Required_Hardware) are followed.

## Assessment notes
The introduction video will give a good overview of the intentions of the project. For ease in idetifying the work we have created below is a list of modifications/additions and a breif description.
* Models - we have created our own brick and table URDF files in order to help the simulation run more smoothly
* Model_Spawn – loosely based on the demo but mostly original with modifications to allow for unlimited spawning
* Pick_n_Place - based on demo with the added lift function allowsing DENIRO to clear the height of the structure
* House_Builder – completely original functions used to create a list of target brick coordinates in the correct order
* Control_Functions – using multiple resources the functions here get feedback from robot sensors
* House_of_Cards – the main function which calls upon functions from all other files to execute the task
 
