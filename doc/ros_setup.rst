Spot ROS Diver Setup
====================

The ROS driver was created and tested on Kinetic, but it should work with Melodic as well.

Building the driver from source
-------------------------------

As the driver hasn't been released yet, it must be built from source.  This requires a source workspace on the ROS PC.

.. code:: bash

  mkdir -p ~/catkin_ws/src

Setup the workspace so it knows about your ROS installation

.. code:: bash

  cd ~/catkin_ws/src
  source /opt/ros/kinetic/setup.bash
  catkin_init_workspace

Clone the spot_ros repository into the workspace

.. code:: bash

  cd ~/catkin_ws/src
  git clone https://github.com/dniewinski/spot_ros.git

Use rosdep to install of the necessary dependencies

.. code:: bash

  cd ~/catkin_ws/
  rosdep install --from-paths src --ignore-src --rosdistro kinetic -y

Once all the necessary packages are installed, build the packages in the workspace

.. code:: bash

  cd ~/catkin_ws/
  catkin_make

Source your newly built workspace and the packages inside

.. code:: bash

 	source ~/catkin_ws/devel/setup.bash
