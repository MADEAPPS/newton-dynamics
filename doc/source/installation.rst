.. _installation:

Installation
============

You can build Newton with- or without graphical demos. For the core library (ie
no graphical demos) you only need a C++ compiler, CMake, a Git client, and the
`TinyXML <http://www.grinninglizard.com/tinyxml/>`_ library.

The sections below outline the process for the both options.


Linux
-----

This section assumes Ubuntu 16.04, but the procedure is similar for other Linux
flavours. To prove it, here are the Dockerfiles for some of them:

* :download:`Ubuntu 14.04 <Dockerfile_ubuntu1404>`
* :download:`Ubuntu 16.04 <Dockerfile_ubuntu1604>`
* :download:`Fedora 22 <Dockerfile_fedora22>`
* :download:`Fedora 23 <Dockerfile_fedora23>`


Core Library Only
+++++++++++++++++

.. code-block:: bash

   apt-get install build-essential cmake git libtinyxml-dev
   git clone https://github.com/MADEAPPS/newton-dynamics.git
   cd newton-dynamics
   mkdir build && cd build
   cmake -DNEWTON_DEMOS_SANDBOX=OFF .. && make


Library + Demos
+++++++++++++++

.. code-block:: bash

   apt-get install build-essential cmake git libtinyxml-dev
   apt-get install libwxgtk3.0-dev libfreetype6-dev libopenal-dev libglew-dev
   git clone https://github.com/MADEAPPS/newton-dynamics.git
   cd newton-dynamics
   mkdir build && cd build
   cmake -DNEWTON_DEMOS_SANDBOX=ON .. && make

The demos are all contained in a single executable. To start it from the
``build`` directory type:

.. code-block:: bash

   ../applications/demosSandbox/demosSandbox 


Other Systems
-------------
If you know how to compile and install Newton on any other system please submit
a pull request.


Questions/Problems
------------------

* Newton does not compile under latest Fedora 24
