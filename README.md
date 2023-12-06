# jango
This is the repository containing docs and code related to the dual arm mobile manipulator system- jango

## Docker container 
# Description 
The docker container contains a built ROS workspace with all the ROS packages running on the computer on the dual arm manipulator. It is useful to have this on a local machine for two reasons. 
1. To access the gazebo sim of the robot + moveit 2. To run moveit on the localmachine using RVIz 


#Usage 
The instructions below are tested for Ubuntu 20.04
Clone the repository 
```
git clone https://github.com/uoft-cs-robotics/jango
cd jango 
```
Install docker.io and docker-compose and depending on the version of docker-compose run either, and make sure docker is added to sudo group in ubuntu- 
```
docker-compose -f docker-compose -f docker/docker-compose-gui.yml build
```
or 
```
docker compose -f docker-compose -f docker/docker-compose-gui.yml build
```

This builds the image to create the container, run- 
```
docker-compose -f docker-compose -f docker/docker-compose-gui.yml create
```
or 
```
docker compose -f docker-compose -f docker/docker-compose-gui.yml create
```
To start the container run 
```
docker-compose -f docker-compose -f docker/docker-compose-gui.yml start
```
or 
```
docker compose -f docker-compose -f docker/docker-compose-gui.yml start
```
Our docker container is now running, we need to now access a terminal inside the container using- 
```
docker exec -it docker_jango bash
```
