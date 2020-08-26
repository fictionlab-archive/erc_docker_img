# ERC Docker Image

This repository contains an example Dockerfile that can be used to build a docker image that will be run on Leo Rover (ERC edition). \
It can be used by the teams as a template for developing their custom software for the ERC competitions.


## Building

Login to the [Freedom Robotics App](https://app.freedomrobotics.ai/), click on **Add Device** -> **Quick Create** and copy the URL address from the command (not the whole command).

Now execute the following command (replace `<YOUR_URL>` with the address you copied):
```
docker build -t erc_img --built-arg FREEDOM_URL=<YOUR_URL> .
```

## Lanching

TODO