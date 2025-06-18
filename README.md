# quadruped_controller_mujoco
## Docker environment for host computer
```
git clone https://github.com/HariP19/docker-ros-dev.git
```
### Set up the dockerfile
#### Edit the `build_image.sh`
```
cd ~/docker-ros-dev/docker_scripts
```
```
vi build_image.sh
```
Edit the parameter DOCKERFILE_PATH_20 in the file to:
```
DOCKERFILE_PATH_20="./docker_build/u20/Dockerfile"
```
#### Edit the `run_image.sh`
```
cd ~/docker-ros-dev/docker_scripts
```
```
vi run_image.sh
```
Edit the parameter IMAGE_NAME in the file to:
```
IMAGE_NAME="harip19/ubuntu20.04:dtc_isaacgym_v3"
```
### Building the Dockerfile

Run the `build_image.sh` in the docker scripts directory. 
```bash
cd docker_scripts
./build_image.sh
```
note: Always build run the script from the docker_scripts directory as it uses relative path. 

### Viewing the built images

Run the following commands to verify if the image is correctly built.
```bash
docker images
# REPOSITORY            TAG                       IMAGE ID       CREATED        SIZE
# harip19/ubuntu18.04   ros_melodic               40d42c7b9f74   23 hours ago   7.93GB
# harip19/ubuntu20.04   ros_noetic                1e2dc55d3474   2 days ago     9.17GB
```

### Starting the Container
Run the `run_image.sh` in the docker_scripts directory
```bash
cd docker_scripts
./run_image.sh
```

### Joining the Container from new Terminal
Run the `join_container.bash` from the docker_scripts directory
```bash
cd docker_scripts
./join_container.sh
```
### Reference

- [BruceChanJianLe/docker-nvidia-ubuntu-ros](https://github.com/BruceChanJianLe/docker-nvidia-ubuntu-ros.git)

## Build the workspace
```
catkin_make --pkg unitree_legged_msgs
source devel/setup.bash

catkin_make
```
## Deployment
In the first terminal:
```
cd ~/quadruped_controller_mujoco
source devel/setup.bash

roscore
```
In the second terminal:
```
cd ~/quadruped_controller_mujoco
source devel/setup.bash

rosrun unitree_mujoco unitree_mujoco
```
In the third terminal:
```
cd ~/quadruped_controller_mujoco
source devel/setup.bash

rosrun quadruped_controller mit_controller a
```
