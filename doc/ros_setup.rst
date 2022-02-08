Spot ROS Computer Setup
=======================

.. note::
    This driver has been tested using the steps below with Spot SDK version 2.3.5.  Any version after this may have unexpected behviour.

The ROS driver was created and tested on Kinetic and Melodic

Setup Spot Core
---------------

If you have a Spot Core you can either use the latest `Melodic ISO <https://packages.clearpathrobotics.com/stable/images/latest/melodic-bionic/amd64/>`_,
set it up with `BalenaEtcher <https://www.balena.io/etcher/>`_, and install it onto the Core, or you can use the official
SpotCORE image from Boston Dynamics.

If you are using a Jetson, follow the `Jetson Setup Instructions <https://docs.nvidia.com/sdk-manager/install-with-sdkm-jetson/index.html>`_

Once your backpack PC is setup, all steps below are to be followed on that PC

Installing Dependencies
-----------------------

.. note::

    If you are using Clearpath's Melodic ISO you can skip immediately to the last step, "installing the dependencies
    for building the ROS driver for Spot" as the ISO will automatically add the ROS and Clearpath apt and rosdep
    sources for you.

If you are using the official SpotCORE image, you will need to add the ROS package sources:

.. code:: bash

    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    sudo apt install curl # if you haven't already installed curl
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

Then add Clearpath's package sources:

.. code:: bash

    sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
    sudo apt update

Install the essential ROS packages:

.. code:: bash

    sudo apt install ros-melodic-ros-base python-rosdep

Configure ``rosdep`` with both the official ROS and Clearpath sources:

.. code:: bash

    sudo rosdep init
    sudo wget https://raw.githubusercontent.com/clearpathrobotics/public-rosdistro/master/rosdep/50-clearpath.list -O /etc/ros/rosdep/sources.list.d/50-clearpath.list
    rosdep update

Finally, install the additional dependencies needed to built the ROS driver for Spot:

.. code:: bash

    sudo apt update
    sudo apt install -y python3-pip bridge-utils git
    pip3 install cython
    pip3 install bosdyn-client bosdyn-mission bosdyn-api bosdyn-core
    pip3 install empy

Setup Networking with ``ifupdown``
------------------------------------

.. note::

    If your computer uses Netplan, please scroll down for instructions on configuring the network using
    ``netplan.io``

Replace the `/etc/network/interfaces` file with the one below

.. code:: bash

  auto lo br0 br0:0 br0:1
  iface lo inet loopback

  # Bridge together physical ports on machine, assign standard Clearpath Robot IP.
  iface br0 inet static
    bridge_ports regex (eth.*)|(en.*)
    address 192.168.131.1
    netmask 255.255.255.0
    bridge_maxwait 0

  # Dedicated port for spot
  iface br0:0 inet static
    address 192.168.50.1
    netmask 255.255.255.0

  # Also seek out DHCP IP on those ports, for the sake of easily getting online,
  # maintenance, ethernet radio support, etc.
  iface br0:1 inet dhcp

.. note::

    If you find that br0:0 is not coming up automatically on startup, you can add ``ifup br0:0`` to
    /etc/rc.local.  If /etc/rc.local doesn't exist, create it and run ``sudo chmod +x /etc/rc.local`` to
    make it executable.


Setup Networking with ``netplan.io``
------------------------------------

.. note::

    If your computer uses the older ``/etc/network/interfaces`` file and the ``ifupdown`` package to manage
    network interfaces, please scroll up for instructions on configuring the network

Remove any wired network configuration files from ``/etc/netplan``.  Wireless configuration files may be retained.
Create the file ``/etc/netplan/50-ethernet-bridge.yaml`` with the following contents:

.. code-block:: yaml

    # Bridge together all physical ethernet ports and allow them to operate simultaneously on:
    # - 192.168.131.1/24 for ROS
    # - 192.168.50.1/24 for communicating with the Spot base platform
    # - dhcp for wired external internet access
    network:
  version: 2
  renderer: networkd
  ethernets:
    bridge_eth:
      dhcp4: no
      dhcp6: no
      match:
        name: eth*
    bridge_eno:
      dhcp4: no
      dhcp6: no
      match:
        name: eno*
    bridge_enp:
      dhcp4: no
      dhcp6: no
      match:
        name: enp*
    bridge_enx:
      dhcp4: no
      dhcp6: no
      match:
        name: enx*
  bridges:
    br0:
      dhcp4: yes
      dhcp6: no
      interfaces: [bridge_eth, bridge_eno, bridge_enp, bridge_enx]
      addresses:
        - 192.168.50.1/24
        - 192.168.131.1/24

Run ``sudo netplan generate; sudo netplan apply`` or reboot the computer to apply the new network settings.  Run
``ip a`` and verify that ``br0`` has IP addresses on the 192.168.131.0/24 and 192.168.50.0/24 subnets.


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
  git clone https://github.com/ros/geometry2 --branch 0.6.5

Use rosdep to install of the necessary dependencies

.. code:: bash

  cd ~/catkin_ws/
  rosdep install --from-paths src --ignore-src -y

Once all the necessary packages are installed, build the packages in the workspace

.. code:: bash

  cd ~/catkin_ws/
  catkin_make --cmake-args -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so

Source your newly built workspace and the packages inside

.. code:: bash

  source ~/catkin_ws/devel/setup.bash
