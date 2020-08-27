# ERC Docker Image

This repository contains an example Dockerfile that can be used to build a docker image that will be run on Leo Rover (ERC edition). \
It can be used by the teams as a template for developing their custom software for the ERC competitions.

This example image starts the Freedom agent which connects to your account at [Freedom Robotics App](https://app.freedomrobotics.ai/), as well as a launch file from an example package which start a ROS node that let's you save images from topics to PNG files.

## Building

Login to the [Freedom Robotics App](https://app.freedomrobotics.ai/), click on **Add Device** -> **Quick Create** and copy the URL address from the command (not the whole command).

Now execute the following command as the `root` user (replace `<YOUR_URL>` with the address you copied):
```
docker build -t erc_img --build-arg FREEDOM_URL=<YOUR_URL> .
```

## Lanching

To run the image, simply type:
```
docker run -it --net=host --name erc_img erc_img
```
If you want to use it with ROS running on another machine, pass the `ROS_IP` and `ROS_MASTER_URI` variables:
```
docker run -it --net=host -e ROS_IP=<YOUR_IP> -e ROS_MASTER_URI=<MASTER_URI> --name erc_img erc_img
```

The Freedom agent should start and you should see the device connected on the [Freedom Robotics App](https://app.freedomrobotics.ai/).

To test the image with the simulated Rover, simply start the [simulation](https://github.com/fictionlab/erc_sim_ws) on the host machine or on a docker container started with the `--net=host` option.

## Using the example features

The Dockerfile properly configures the image to permit SSH login. \
On the Freedom Robotics App, click on **Settings** -> **Remote SSH** -> **Enable remote SSH**. This will open an SSH tunnel that will let you login to your container from the Internet. \
To login, paste the resulted command into your terminal. When asked for password, type `root`.

The `start.sh` script that is executed by the docker image, configures the environment, so when you login, you can already run ROS commands. \
You can start by listing the available topics:
```
rostopic list
```
You should see the `/image_saver/save` topic which is spawned by the `image_saver` node from the `erc_example` package. \
The node expects Image topic names on that topic and upon receiving a message, saves a single Image from the topic to a PNG file inside the container. \
If you have the simulation running, you can try to save an image from the hazcam by executing:
```
rostopic pub -1 /image_saver/save std_msgs/String "data: '/camera/image_raw'"
```
An image should be saved inside the `/root/.ros` directory.

You can also use Freedom Robotics App to send messages on this topic, by going to **Settings** -> **Send command**.