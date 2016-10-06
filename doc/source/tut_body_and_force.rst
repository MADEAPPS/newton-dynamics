Basic Rigid Bodies And Forces
=============================

Purpose: create a spherical body, apply a force, and track its motion over time.

In ``addSphereToSimulation`` we create a sphere and install a *force callback*.
Newton will unconditionally trigger that callback before it progresses the
simulation. Similarly, we can install a callback for whenever a body moves.
However, this callback will only be called if the object has actually moved.

The expected outcome is that the body moves 0.5 meters, because its mass is 1kg
and we apply a force of 1N for one second.

Here is the :download:`code <./tut_body_and_force.cpp>`:

.. literalinclude:: tut_body_and_force.cpp
    :linenos:
    :language: c


Linux
-----
To compile, link, and run the program type:

.. code-block:: bash
    :linenos:

    $ g++ tut1.cpp -o tut1 -I ../../coreLibrary_300/source/newton/ -L ../../build/lib/ -lNewton -lpthread -Wl,"-rpath=../../build/lib/"
    $ ./tut1


Other
-----
Please submit pull request for instructions.
