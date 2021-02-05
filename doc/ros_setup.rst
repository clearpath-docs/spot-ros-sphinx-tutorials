Spot ROS Diver Setup
====================

The ROS driver was created and tested on Kinetic, but it should work with Melodic as well.

Installing Dependencies
-----------------------

.. code:: bash

  sudo apt update
  sudo apt install -y python3-pip
  pip3 install cython
  pip3 install bosdyn-client bosdyn-mission bosdyn-api bosdyn-core

Building the Driver from Source
-------------------------------

As the driver hasn't been released yet, it must be built from source.  This requires a source workspace on the ROS PC.

.. code:: bash

  mkdir -p ~/catkin_ws/src

Setup the workspace so it knows about your ROS installation

.. code:: bash

  cd ~/catkin_ws/src
  source /opt/ros/melodic/setup.bash
  catkin_init_workspace

Clone the spot_ros repository into the workspace

.. code:: bash

  cd ~/catkin_ws/src
  git clone https://github.com/clearpathrobotics/spot_ros.git

Use rosdep to install of the necessary dependencies

.. code:: bash

  cd ~/catkin_ws/
  rosdep install --from-paths src --ignore-src -y

Once all the necessary packages are installed, build the packages in the workspace

.. code:: bash

  cd ~/catkin_ws/
  catkin_make

Source your newly built workspace and the packages inside

.. code:: bash

  source ~/catkin_ws/devel/setup.bash
