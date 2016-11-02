# The X8 Simulink simulator
**In order to use this project you MUST use git**. I.e.: do not download this repository as a zip archive. No git -> no simulator
The master branch contains the newest version of the simulator.
dev includes files used by KG to generate the model parameters.

**Make your own branch if you plan on modifying stuff**

### Dependencies:
- Matlab 2015a or newer
- GNC toolbox from [the MSS toolbox](marinecontrol.org/download.html "Marine Control")
- Matlab Aerospace toolbox (included in NTNU emplyee licence)

### Guidelines:
0. See X8_demo.slx to get an idea of how to use this simulator. Initialize by running initX8_demo.m
1. clone this repository
2. Open the library file X8_lib.slx
3. Drag the *UAV model* block into your project. Set the block parameters according to your situation. *bus_defintions.m* has to be run before start the simulation, but no other variables. If you ever have problems with the *UAV model* in your simulink project saying "Unresolved link", you might have to drag the file from the library and into your project again.
<!---
? 6. Set *Extraneous discrete derivative signals* to 'none' in the Diagnostics tab in the *Configuration Parameters* panel.
-->

### TODO:
1. Verify initializations
2. Change X8_trim.slx to use library block. Problem with quaternion norm.
3. make FlightGear library -> how to initialize IP?

### Variables affected by running initX8_demo.m
**Name                   Size            Bytes  Class           Attributes**
Acceleration           1x1               364  Simulink.Bus              
AirData                1x1               815  Simulink.Bus              
AngularVelocity        1x1               352  Simulink.Bus              
Attitude               1x1               368  Simulink.Bus              
Control                1x1               445  Simulink.Bus              
Position               1x1               364  Simulink.Bus              
Quaternion             1x1               345  Simulink.Bus              
References             1x1               386  Simulink.Bus              
States                 1x1               646  Simulink.Bus              
U_trim                 4x1                32  double                    
Velocity               1x1               352  Simulink.Bus              
X_trim                18x1               144  double                    
h_ref                  1x1                 8  double                    
input_source           1x1                 8  double                    
input_source0          1x1                 8  double                    
input_step_time        1x1                 8  double                    
lat0                   1x1                 8  double                    
lon0                   1x1                 8  double                    
tout                 329x1              2632  double                    
