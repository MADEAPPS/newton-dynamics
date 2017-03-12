Basic Rigid Bodies And Forces
=============================

Purpose: create a spherical body, apply a force, and track its motion over time.


We are now adding two more functions. The first, ``addSphereToSimulation``
creates a sphere in the Newton world and parameterises it (mass, inertia). It also
installs the second new function ``cb_applyForce`` as a callback. Newton will
trigger that callback whenever it moves the body. It will *not* trigger it if
the body is already at rest.

Inside the callback we query the state matrix and print the current position.
Furthermore, we apply a force along the ``y`` axis, which makes the body
accelerate in that direction. Since that force is 1N, and the simulation runs
for one second, the body must move exactly 0.5 Meters.

Here is the :download:`code <./tut1.cpp>`:

.. literalinclude:: tut1.cpp
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


Questions/Problems
------------------

* What is the third parameter to ``NewtonCreateDynamicBody``?
* What is ``shapeID``?
