# README
interface, install steps, usage instructions, etc.

### Scenic Scenarios
#### Holds .scenic files (these define sim scenes like roads, vehicles, map environment)
Used as inputs for generating simulation AV environment

### Interface
####  Contains Python scripts that serve as the interface between your.scenic inputs and the simulator (AWSIM)
Ex. scenic_to_awsim.py
 - Parse scenic files
 - Convert them into formats readable by AWsim
 - Possibly launch simulations via ROS2 or shell

### Set up
Install full version AWSim (does not work with demo) \
follow instructions for setup from AWSim \
`https://autowarefoundation.github.io/AWSIM-Labs/pr-59/GettingStarted/SetupUnityProject/`

From the project file directory ('216Project') run the scenic_driver.py file \
`python3 -m awsim.interface.scenic_driver`