# Simulation Base Class and displaySimTools
Designed by: Raphael Fortuna  
Additions: Shubham Goda

- [Summary and Outcomes](#summary-and-outcomes-)
- [Terminology](#terminology-)
- [How to Use](#how-to-use-)
- [Background Research](#background-research-)
- [System Information](#system-information-)
  - [Information Sent](#information-sent-)
  - [Information Received](#information-received-)
  - [Dependencies](#dependencies-)
- [Algorithms and Designs](#algorithms-and-designs-)
  - [Algorithm(s) in Use with Rationale](#algorithm-in-use-with-rationale-)
  - [Previous or Unused Algorithms with Rationale](#previous-or-unused-algorithms-with-rationale-)
- [Performance](#performance-)
- [Future Work](#future-work-)

## Summary and Outcomes <a name=“summary-and-outcomes”></a>
Used in the Shell Eco-marathon Autonomous Programming Competition Summer of 2020 and Winter 2021. 
simulationBaseClass is what any simulation or display inherits as to have as built-in compatibility and reduce code duplication throughout the system and make it easier to develop new simulations.
displaySimTools is an interface with matplotlib that makes it easier to do repetitive and syntax heavy operations with simple functions and operations and is what powers simualtionBaseClass.

Runs in python 2.7 and 3.7 (though with new updates some things may need to be updated for full 2.7 support).  

[Link of car driving in the virtual environment](https://drive.google.com/file/d/1jL_li86iJGSqepNmJrqMZeN8E8W6ZzqX/view?usp=sharing)

## Terminology <a name=“terminology”></a>
None  

## How to Use <a name=“how-to-use”></a>
simulationBaseClass:  
Create an instance of some base class with the simulation contract, display config map file name as the arguments and then run display full sim. After that it runs.
If you want to add more buttons you have to add the button with the proper syntax to the simulation config, examples of this of which can be found in SimMainConfig, and then for each button, slider, and textbox, link the function part of the dictionary that holds the GUI element information. This will make sure that when you interact with the GUI element it updates whatever function is attached to it.  
A simulationConfig address (no ()), displayConfig address (no ()), map file name, and path name are the required inputs.  
simulationBaseClass cannot be run and must be inherited and run by another function.  
The goal radius, goal boost radius, and waypoint radius can all be changed in the config as well as the shiftX and shifty that are used to move the graph.  

displaySimTools:  
Create an instance of displayTools with a displaySimConfig, run setUpPlot with needed parameters, run loadWindow with needed parameters, run loadGUI with needed parameters (not required), run plotShapes with needed parameters, run showPlot. That’s it, GUI elements and shapes are in specific formats with some listed in [System Information](#system-information-) and if not, are specified in displaySimTools under the function that adds it.  
You can also add dots and lines to the graph with plotDots(x, y, color), save photos of the graph with saveGraph(path), and close the graph with closePlot().  

## Background Research <a name=“background-research”></a>
Used matplotlib wiki for specific matplotlib syntax, [link]( https://matplotlib.org/3.1.1/index.html)  

## System Information <a name=“system-information”></a>
Here is a list of behaviors simulationBaseClass has:  
* All simulation environments and GUI elements are stored in the MainConfig SimulationConfig and are to be added there. GUI functionality is added in Simulations and is linked to the stored GUI element in populateGUI and added to the plot  
* displayFullSimulation iself is not dynamic and only starts and displays the simulation, the button functions added are what make the simulation dynamic by updating the plot when needed with loadWindow and populateGraph  
* Graph window settings are available in the config as well as which variables are to be ignored by debug  
* setup, populateGUI, and populateGraph all need be implemented by the simulation that inherits simulationBaseClass, but the basic GUI elements and graph elements can be shown with populateBaseGUI and populateBaseGraph  
* onClick will likely be redefined so check onClick to see what is used for filtering points and dragging obstacles  

Here is a list of behaviors displaySimTools has:  
*  setUpPlot - creates a new plot instance  
* loadWindow - loads the basic window settings for the plot  
* loadGui - only needs to be run once when starting a simulation plot  
* showPlot - shows the plot, only way to show the plot  
* plotShapes plots arrows, rectangles, and circles in specific formats as mentioned in plotArrow, plotRectangle, and plotCircle  
* plotDots – used to plot dots, WIP
* closePlot – closes the plot
* addElement (TextBox, Rectangle, Slider, Button) - adds a single element at a time  
* plotRectangle - plots multiple rectangles from a list of rectangles that all have the same color  
* plotCircle - plots a single circle at a time  
* plotArrow - plots a single arrow at a time  
* plotImage - plots an image from a file  
* clickGraph – used to send click information to the clicking function in a simulation  
* moveGraph – used to send mouse movement information to the move function in a simulation  
* userResized – used to send resizing information to the resizing function in a simulation   
* figSize – gets graph size  
* saveGraph – saves a picture of the graph  

In the config for displays, there are options to enable clicking, moving, and resizing as well as if the plot should remain hidden. There are also other options but those are the most important ones

### Information Sent <a name=“information-sent”></a>
Varies by simulation use, but all things to be plotted, graphed, or GUI elements to be added are sent to an instance of displaySimTool’s displayTools.  
displaySimTools show a plot.  

### Information Received <a name=“information-received”></a>
simulationBaseClass receives the address of the simulationConfig that contains the simulation’s config and GUI button information and displaySimConfig as well as the path and map file name that is to be loaded.  
displaySimTools receives arrows, rectangles, and circles to be plotted from simulations as well as the GUI buttons to be added. It also receives the displaySimConfig class for the simulation that contains the display’s configs.  

### Dependencies <a name=“dependencies”></a>
simulationBaseClass:  
* Libraries: abc  
* Files: routePlanner, ConfigLoader, displaySimTools, toolbox, routePlanner *

displaySimTools:  
* Libraries: Matplotlib.pyplot, matplotlib.patches, matplot.lib.widgets – button, slider, textbox, matplotlib.image – mping, matplotlib  
* Files: None  

## Algorithms and Designs <a name=“algorithms-and-designs”></a>
### Algorithm(s) in Use with Rationale <a name=“algorithm-in-use-with-rationale”></a>
simulationBaseClass:  
Maps are shifted if needed by specific amount specified in the config and this can be applied to a single map or multiple maps and is use when loading ramps as well as saving maps and involves iterating through all obstacles, goals, and waypoints to shift their position.  

Moving obstacles: made by Shubham Goda, more information to be provided later

displaySimTools:  
The algorithms here are for modularity and for setting and doing things with matplotlib in an easier format and have been mentioned in [System Information](#system-information-).  
This is a lot of abstraction that takes place for GUI elements and how a GUI elements is tied to a function. The simulation class links the address of a function to an element in a dictionary form which is passed to create a GUI element in displaySimTools and then uses the name of the element to link a function to it that also passes through the dictionary. Since this function address came from the simulation, the GUI is able to access the simulation functions and makes it work seamlessly without hardcoding.

### Previous or Unused Algorithms with Rationale <a name=“previous-or-unused-algorithms-with-rationale”></a>
Previously before displaySimTools, all matplotlib functionality was unique and hard-coded in, I wanted to avoid this because it made the code harder to work with and if someone with less matplotlib experience wanted to make a simulation, it was very complicated.  

Before displaySimTools existed, to use matplotlib, the entire structure of a particular plot usually had to be built from scratch and hard coded every time. However, now with displaySimTools, all the matplotlib support framework has been created, so for waypointGeneration, all that I had to do was build the logic and the functions needed to actually interact with the waypoints, edges, and obstacles based off of mouse clicks. This made development faster and allowed for code reuse. I also added an abstract interface inside of displaySimTools so that any new simulations or general matplotlib tools can use this abstract class to have a framework for what is needed to run the simulation. This abstract interface later become SimBaseClass.  
Also, previously the middle mouse button was used to click plot buttons to avoid interacting with the graph area, but this changed to checking if the mouse click on the matplotlib window was within a certain range that defined the plot to make the program easier to use.   
Previously for simulationBaseClass, only some functions were included but new simulations would have to repeatedly set up resizing and clicking. This led to a lot of code duplication so we changed it so that the simulationBaseClass would automatically connect to the resizing and clicking functionalities to displaySimTools and would instead be dependent on a displaySimConfig for each simulation for if clicking, resizing, and moving elements should be enabled. I also made it so these functions are also automatically connected to display simulation to avoid having to hard code more functionality and instead leverage flexibility.  

## Performance <a name=“performance”></a>
displaySimTools:  
* Varies by what is being simulated and what each GUI button updates  

simBaseClass:  
* Varies by what is being simulated and what each GUI button updates, clicking too many buttons can cause stack overflow since each button is interrupting the previous button’s operation    

## Future Work <a name=“future-work”></a>
displaySimTools:  
* Fix plotting lines bug with plotted lines not being shown on the correct axes/figure  


# Created Simulations and Displays for Motion Planning Projects
Designed by: Raphael Fortuna  
Additions: Shubham Goda

- [Summary and Outcomes](#summary-and-outcomes--1)
- [Terminology](#terminology--1)
- [How to Use](#how-to-use--1)
- [Background Research](#background-research--1)
- [System Information](#system-information--1)
  - [Information Sent](#information-sent--1)
  - [Information Received](#information-received--1)
  - [Dependencies](#dependencies--1)
- [Algorithms and Designs](#algorithms-and-designs--1)
  - [Algorithm(s) in Use with Rationale](#algorithm-in-use-with-rationale--1)
  - [Previous or Unused Algorithms with Rationale](#previous-or-unused-algorithms-with-rationale--1)
- [Performance](#performance--1)
- [Future Work](#future-work--1)

## Summary and Outcomes <a name=“two-summary-and-outcomes”></a>
This is an entry for all simulations and displays developed using displaySimTools and simBaseClass.  
This includes:  
* bulkMapGenerationSim – used for generating random obstacle maps in bulk  
* carCalibrationSim – used to calibrate the potential field of a vehicle as well as a rough estimate of PID values output
* rrtSim – used to test the and visualize the RRT path generation and see how it reacts to obstacles being moved around  
* waypointGeneration – used extensively in competitions, lets us see what an autoMap looks like and lets us create maps. It also lets us visualize a previous trial’s path, visualize a previous trial’s path driven by a little dot, and see our shortest path algorithm calculated routes  

## Terminology <a name=“two-terminology”></a>
**Waypoint** - a node on the map  
**Edge** - a path connecting two waypoints  
**clickCache** - a way to temporarily store information about what was clicked  
**autoMap** - the map format used by all simulations and displays, except for carCalibrationSim, and is located inside of routePlanner and contains information about the waypoints, edges, obstacles, and goals of a certain map.  

## How to Use <a name=“how-to-use”></a>
To run a simulation, a config for the simulation and the display as well as any other parameters including the map and path must be passed to the instantiation of the simulation (add map). Any simulation must instantiate the initialization of the base class which allows for all the required variables and configurations to be set. A few functions must also be implemented including the setup function and populate GUI function and populate graph function.  
In both of these functions, a base function is available to add frequently used GUI buttons and elements of the plot, populateBaseGUI and populateBaseGraph, and remove a lot of duplicated code within the simulations.  
After setting all of this up, as well as the super for the base class initialization, instantiate the class and run the function displayFullSimulation. Running this function will begin the simulation and, if you have all your buttons set up correctly and all the functions linked as shown in examples that can be found in the simulationBaseClass and other simulations mentioned in this wiki entry, then they will all update automatically as you change and interact with the elements on the display.  
If no map is given then a blank map will be generated by default.  
The built in buttons include Next Map, Previous Map, Convert Map, Export to JSON, Reset, Debug, Random, Click Toggle, and Drag Toggle.  

**Next Map** - is used to go to the next map in the list of maps  
**Previous Map** -is used to go to the previous map and the list of maps  
**Convert Map** - is used to convert the map from the auto map format to the road node format used by the path optimizer to ensure that the map is being saved and loaded correctly  
**Export to JSON** - this exports the list of maps to the JSON file located in the path folder. the name must be specified. the map information will also be printed  
**Reset** - is used to reload the map to its original state and erases any changes made  
**Debug** - is used to print all the values of the simulation as well as any children of the simulation and it can be helpful when debugging or looking for certain errors or variable changes. a list of skipped variables can be set in the simulation's config if you would like to skip a large class, otherwise if there is a class instantiated by your simulation, all of its elements will be printed  
**Random** - this is used for testing purposes in case you would like to add a function and see how it reacts without having to add a formal button to the system  
**Click Toggle** - is used to toggle the cursor on and off for clicking on the graph. this can be used to make sure you don't accidentally click something on the graph  
**Drag Toggle** - this is used to toggle the cursor on and off for clicking on obstacles and being able to drag them around. when this is turned on you will not be able to do anything with the mouse except for drag obstacles around, unless you have over overloaded the on click function and have your own  

All simulations have configs in SimMainConfig
Example running for running the waypointGenration simulation:
#create the path
path = toolbox.generateFolder('wayPointGenTesting', False)

#SimMainConfig.waypointGenerationConfig is for the simulation, notice how it is not created, but the address is stored  
simConfig = SimMainConfig.waypointGenerationConfig  

#SimMainConfig.waypointGenerationDisplayConfig is for the display, notice how it is not created, but the address is stored  
displayConfig = SimMainConfig.waypointGenerationDisplayConfig  
#map file  
mapFile = 'simulationAssets/ShellMapFinal.json'  
#optional image  
image = ''  

#trial data  
trialData = 'simulationAssets/odom_data.xls'  
  
newSim = waypointGen(simConfig, displayConfig, path, mapFile, image,  trialData)  
newSim.displayFullSimulation()  

### bulkMapGenerationSim:

Link: [Video Demo](https://drive.google.com/file/d/1BscgbVE3Q4tqx94F6pZhXlt4LIGGYKbG/view?usp=sharing)
4 buttons were added:  
* Except - used to accept a map when it is created  
* Reject - used to reject a map when it is created, can also be used at any time to delete the current map shown, but this must be confirmed in the console  
* Generate - used to generate a new random map  
* Go to Last - used to go to the last map in the map set  
Obstacle density can be changed in SimBaseClass under BMGSimConfig with obstacleDenominator  
It will avoid a specified protected area around to reduce instant collisions when making maps. This specific protected area can be changed in the BMGSimConfig protectedArea  


### carCalibrationSim: 

Link: [Video Demo](https://drive.google.com/file/d/1vlHwbK-w0EzoMNNbu0Df33J59CybBojo/view?usp=sharing)
2 buttons were added with 7 sliders:  
* Vehicle Control Update (VCUpdate) - this toggles if the pa d updates every over time or resets each time a value is changed
* Change Mode - this is used to cycle through the different scenarios  
*  Max Potential - used to change the max potential of the potential field controller  
*  Attraction (Attrac) - used to change the attraction variable for the potential field controller  
*  Repulsion Multiplier (RepMult) - used to change the repulsion multiplier variable for the potential field controller  
* Repulsion (Rep) - base used to change the potion variable for the potential field controller  
*  Velocity Proportion (VProp) - used to change the proportional value of the vehicle control velocity PID  
* Velocity Integral (VInt) - used to change the integral value of the vehicle controller PID  
* Velocity Derivative (VDer) - used to change the derivative value of the vehicle controller PID  

### rrtSim: 

Link: [Video Demo](https://drive.google.com/file/d/1f5oTBVPYx31XedULWN3a-fRiKUiVNY7r/view?usp=sharing)
No extra buttons are added; however, moving the obstacles will update the RRT path in real-time.   

### waypointGeneration: 

Link: [Video Demo](https://drive.google.com/file/d/1tgiTUIcpJ6V4D55hE6M9NuoCrNqSdUdS/view?usp=sharing)
7 buttons were added with 1 slider:  
* Show Car Path - shows the path of a previous run using the excel information if it is provided  
* Drive Car - shows the car driving around with header vectors to observe the path of the car, the speed of it driving can be changed by increasing the time slider  
* Obs/WP - is used the switch between way. in place placement and obstacle placement. When in obstacle placement mode, left click on two points on the map to place an obstacle, and right click on any obstacle to deleted. When in waypoint placement, left clicking on the map away from a waypoint will place it, 2 clicks will delete the waypoint and any connected edges, and clicking next to two waypoints will connect an edge between them. If you want to remove an edge between two waypoints, left click or right click on one of the waypoints and then right click on the other waypoint to delete the edge between them.  
* Place Start - when the first point is placed, it is colored red and is the start. This. may not be deleted and may only be moved by clicking the place start button. Any connected edges will remain connected to it when start point is moved  
* Simulate Path - is used to simulate a path on the map using the shortest path algorithm  
* Move forward  - is used to increment the simulated path  
* Move backward - is used to decrement the current simulated path  
* Time - is used to speed up the car when n is driving around or reduce the number of points shown when shown car path is clicked  
* Random - this was not an added button; however, this button has a custom function that allows for you to increment the path automatically and watch it move  

When sending the trial data, make sure row 1 is x position, row 2 is y position, row 4 is heading x, row 5 is heading y, row 7 is heading, row 8 is speed, row 9 is seconds, row 10 is milliseconds  

The distance that constitutes as near when clicking near a waypoint can be changed in the SimMainConfig config under waypointGenerationConfig.nearPoint

## Background Research <a name=“two-background-research”></a>
None.

## System Information <a name=“two-system-information”></a>
Runs off of displaySimTools and the abstract interface simulationBaseClass.  
To learn more about the button format and available configs please see above [Simulation Base Class and displaySimTools](#simulation-base-class-and-displaysimtools)  

### Information Sent <a name=“two-information-sent”></a>
Saves a map in JSON format to a given path using autoMap class in routePlanner, adds any generated files to the path specified  

### Information Received <a name=“two-information-received”></a>
Loads a map file and any other required information like .xls files and images for waypointGeneration

### Dependencies <a name=“two-dependencies”></a>
* Libraries: None  
* Files: displaySimTools, toolbox, routePlanner, MasterSimClass, other files if needed for the specific simulation  


## Algorithms and Designs <a name=“two-algorithms-and-designs”></a>
### General  
For information about SimBaseClass please see above [Simulation Base Class and displaySimTools](#simulation-base-class-and-displaysimtools)  

### bulkMapGenerationSim:
Runs off a map generation function that used to be its own file but was merged into a simulation. It determines how many obstacles should be added base off the map size and obstacleDenominator config and creates random rectangular obstacles in the window area while avoiding the protected area.  

### carCalibrationSim: 
Contains a carState class that it uses with miniRos to run all the potential field and vehicle control updates.  
Has an environmentState class that keeps tracks of goals and obstacles since LIDAR, Ray vectors, and processed obstacles can all be used by the motion planning system.  
Contains own custom map system to support more than 1 obstacle type.  
Contains code for running dynamic simulations that works by updating the carState in reference to controls created by miniROS, applying those changes to the map for the car to “move” and then recalculating the next move over and over again until a goal is reached. This code does not have a physics model and therefore will not work since the car location, speed, and acceleration cannot be updated. With a physics model, it would be able to work.  

### rrtSim (Shubham Goda):
No new algorithms. Runs off SimBaseClass code and updates itself from the RRT Code.

### WaypointGeneration
The algorithms currently used focus on the mouse input and the states of the plot. When the mouse is clicked, 6 different pieces of data are sent, the x and y coordinates for where the mouse is clicked on the plot (referenced here as mouse click location), the x and y coordinates for where the mouse was clicked on matplotlib window, and what button number was clicked - 1 for left, 2 for middle, and 3 for right. By looking at these different pieces of information and if an obstacle or waypoint is being made as well as what is currently in the clickCache, the operation can be determined.  
The logic first checks if the mouse clicked inside the plot and if the left or right click button was clicked. If this is true then it checks if the start waypoint condition set by the “Place Start” button is True to be placed and if it is, places it where the mouse clicked. By checking for if the mouse clicked inside the plot, when clicking buttons, random points are not added to the plot.  
If the start waypoint condition is false, then it checks if it is currently in waypoint placement mode, and if it is, checks if the location of where the mouse clicked is next to one of the waypoints. It does this by finding all the distances between the mouse click location and each waypoint and selecting the smallest. If the smallest is within the near distance, then location is added to the clickCache where it is temporarily saved.  
If the clickCache has two points, then there is a pair, and this means that either this pair will be used to make an edge between two different points if left click was the last click or used to delete an edge between two waypoints if right click was the last click. If the two points inside of clickCache are the same, this means that the points should be deleted. Whenever an edge is deleted, is removed from the edge list. Waypoints themselves do not contain information about edges so no updating is needed there, reducing overhead. The waypoints later get edge information when they are loaded by the routePlanner. If a waypoint is removed, all edges associated with it are also removed. If the mouse click was not near a waypoint this means that this is a new waypoint, so a new waypoint is created at the location of the mouse click.  
If the map is currently in obstacle placement mode, then it uses a similar process for the clickCache, checking that two clicks have been made to create a rectangle from them. The checking process sets a variable, self.obsPair, to True once one point has been picked so that it can be plotted, and sets it to False once an obstacle has been created. To delete an obstacle, it checks if the mouse click location was inside of a current obstacle and if right click was pressed.  
Other functionality also includes making an obstacle from two corners, if an edge should be added or removed since the selection of an edge to be removed could be inverted in relation to the edge in the edges list, and whenever creating edges, if it would be a duplicate edge, it is not created.   
When Drive Car is pressed, the trial data is iterated over with a step of the value of Time and the heading is calculated based on the car’s movements. The calculated heading as well as the heading located in the trial data is shown on the map with a dot that signifies the car that moves around the map.  
When Show Car Path is press, the trial data is again iterated over with a step of the value of Time and the entire path is shown on once.  
When Simulate Path is pressed, the simulation takes information from the MainConfig map information and runs the optimizer on it - which can be either the shortest path or the optimal path based off of the left and right sides of a track - and stores the movements that the car will be doing from 1 node to the next. Then if Move forward is pressed then the path will be iterated on and the next segment of the path will be shown and can be automatically done by pressing Random. This is undone with Move backward. Using this method, path that the car will take can be visualized on whatever map is currently shown.  

### Previous or Unused Algorithms with Rationale <a name=“two-previous-or-unused-algorithms-with-rationale”></a> 
Before SimBaseClass existed, each simulation would have its display, graphing, and buttons repeatedly defined, which made it more complicated to maintain and manage there was a lot of code duplication, with the introduction of the SimBaseClass, is heavily reduced the simulation to the. For one of the newest simulations, RRT simulation, it is a mere 22 lines of code, not including function names, comments, imports, and whitespace. This change has made it easier to create new simulations and displays and speeds up development time.

## Performance <a name=“two-performance”></a>
Loading a map takes less than a second and all changes made to the plot are instantaneous if clicks are made with more than .5 to 1 second in between them, otherwise, the plot becomes overloaded with inputs since loops are used to delete edges. Change may also take more time if there is external code that must be run for data that will be shown on the plot.  
 
## Future Work <a name=“two-future-work”></a>
Updates and general bug fixes  
Bug fix for why plotted lines are not shown but when the image is saved, are saved with the graph  
