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

### Config Templates
####  I'm thinking YAML or JSON templates? Reusable files that define simulation parameters.
Automatically injects these conflicts into the simulator and makes the simulations configurable and reproducible. 


### Launch Scripts
####  Stores ROS2 launch.py files or shell scripts that wrap around the simulator startup processes
Essentially the house for all the automation stuff.
#### Full scenario -> config -> simulation -> logging 

### Docs
####  Contains project documentation
 Helps us both understand each other's code, or anyone who is an onlooker

### Tests
####  If we need it, stores unit tests or integration tests for scenario parsing logic, conversion pipeline, simulator interface.

### README.md
####  Top-level documentation file for the GitHub.
- What this project does
- How to install dependencies
- How to run the code (basic usage)
- File folder structure