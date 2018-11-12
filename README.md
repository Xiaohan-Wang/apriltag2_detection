# Guide to duckietown-rosdocker-template
A template for a Docker container for a ROS Node that supplements the Duckietown ROS core.

This template can be used as a starting point when developing nodes to supplement the Duckietown ROS system leveraging the distributed capabilities of Docker and ROS. The template comes with a basic node `joy_cli` that allows controlling the robot through the terminal. Note that to function properly it requires that the `duckietown/rpi-duckiebot-joystick-demo:master18` and `duckietown/rpi-ros-kinetic-roscore:master18` containers are running on the Duckiebot.

## What's in the template?
The template is set up to build as a `catkin` workspace. That is why there's the `.catkin_workspace` file that signals this to `catkin_make`. When you run `catkin_make` for the first time, it will build two more directories: `/devel` and `/build`. When pushing the code to GitHub, sharing it with others or building a Docker image, the `/devel` and `/build` directories are not necessary. Don't try to reuse them as they contain paths that are specific to the system where they were build. To make sure that `git` doesn't track them, they are included in the `.gitignore` file.

This template does not contain any of the core Duckiebot functionality. It assumes that you run the `duckietown/rpi-duckiebot-joystick-demo:master18` container as a ROS master and via it it can launch, connect and use core nodes, services, etc. This keeps the add-on nodes separate from the Duckiebot core which in turn allows for easy portability and stacking of add-ons.

In the `/src` directory you will find two packages: `duckietown_msgs` and `joy_cli`. Typically, you won't have to edit the `CMakeLists.txt` file. The `duckietown_msgs` containes the ROS message definitions needed to communicate with the other Duckiebot ROS nodes. Therefore in almost all circumstances you will need to use it. The `joy_cli` package contains an example of a very basic ROS node that allows controlling the robot through the terminal. You can see how it is implemented and use that as a basis for your nodes. Once the workspace is built by executing `catkin_make` and the environemnt is set (more on this later), the `joy_cli` node can be started by running (substitute `duckiebot` for your bot's name):

```
$ roslaunch joy_cli joy_cli.launch veh:=duckiebot
```

## How to get started?
Here are the first steps you should take for making your own node.

1. Set up a GitHub repository. Make sure it is empty.
2. Get the contents of this repository into your repository. You can do that by downloading this repository as a `zip` archive or by running this sequence of commands in a terminal:

   ```
   $ git clone --bare https://github.com/duckietown/duckietown-rosdocker-template.git
   $ cd old-repository.git
   $ git push --mirror https://github.com/yourusername/your-repository-name.git
   $ cd ..
   $ rm -rf old-repository.git
   ```

3. Build your directory with `catkin` dor the first time. Navigate to your directory and then run:

   ```
   catkin_make
   ```

You are all set now!


## How to develop and test on my laptop?
It is generally easier to develop your code and debug it if you are working on your computer and not on the Duckiebot. This can be easily done thanks to the distributed nature of ROS. You will still need your Duckiebot to be on and to have the `duckietown/rpi-duckiebot-joystick-demo:master18` container running. Then, you can setup your laptop ROS environment such that it uses the Duckiebot ROS master. Here are the steps to do that (assuming you have ROS Kinetic already).

1. Run `catkin_make` in your node folder if you haven't already.
2. Open a terminal in your node folder (should conatain `devel`, `build` and `src` folders) and run `source devel/setup.bash`. That will setup all the ROS functionality you need in your terminal.
3. Run `export ROS_MASTER_URI=http://duckiebot:11311`, substituting `duckiebot` with your bot's name. This will instruct ROS not to establish a local master but to instead connect to your Duckiebot's master.
4. Use the `export` command to set up any other environemnt variables that your nodes require.
5. Now you can launch your nodes with `roslaunch`, `rosrun`, etc (see the example above).
5. Don't forget to commit your changes to `git` from time to time!

## I am done developing, how can I deploy directly on my Duckiebot?
So far your new nodes were running on your computer. As you can imagine, while easy for development, that is not a great solution for portability. Also, you might be unknowingly _cheating_ with the extra computational resources on your computer. The deployment standard in Duckietown is Docker containers. By building a Docker container from your nodes and testing it with, you can be (almost) sure that your work is truly portable, well-functioning and will be usable by others easily. So how do you do that?

1. Take a look at the `Dockerfile` in your main directory. It was setup for the `joy_cli` example. Update your maintainer label, add explanations on any other environment variables or special options that need to be passed when your container is run, and add commands to install any additional packages your nodes might be needing. Update which node gets launch by default in the `node_launch.sh` file. You can always override that by passing a different command when executing `docker run`. For example you might want a simple `bash` terminal to explore the contents of your container.

2. Build your image. Open a terminal in your main folder and then run `docker build -t dockerhub_username/repository:tag .`. If it fails or freezes try passing the `--no-cache` argument. You can then push your image to DockerHub by running `docker push dockerhub_username/repository:tag`.

3. When your image is already on Dockerhub, you can run it directly on your Duckiebot. Simply `ssh` into it (or use Portainer). Now you can run your the container with:

   ```
   $ docker run -it --network host -e ROS_MASTER_URI='http://duckiebot:11311' -e DUCKIEBOT_NAME='duckiebot' dockerhub_username/repository:tag
   ```

   Remember to add any additional environment variables or options you might need. If you want to execute a command different from the one set in `node_launch.sh`, you can add it at the end of the command. If your conatiner doesn't need an interactive terminal you can omit the `-it` options. Bear in mind that if your node tries to start a graphical interface, you will need to pass a few extra arguments to the command.

You can find your workspace in `/node-ws` in the running container. It is important to never rename this folder as that will destroy a number of links.

## Can I have a development container with git-tracked folder?
Yes, you can! This is sometimes helpful for development. Say you want to quickly change the code in the container as it is running on Duckiebot. You can simply `ssh` in it and make any adjustments necessary. However, now your GitHub repository won't match this. If you want to circumvent that, simply make your `node-ws` directory tracked.

The easy way to do that (before you make changes) is to simply delete the `node-ws` folder from the container and then clone your git-tracked folder instead.

```
$ rm -r /node-ws
$ git clone https://github.com/yourusername/your-repository-name.git /node-ws
$ cd /node-ws
$ catkin_make
```
Now you can use `git push, pull, commit, etc`. However, be careful when removing the directory. Any changes that were not tracked in the repository will be lost!
