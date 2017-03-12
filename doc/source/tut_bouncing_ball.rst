Ball Bouncing Off Surface
=========================

Purpose: specify the initial position of bodies and let them collide.

This simulation creates a sphere 2m above (in y-direction) of a 0.1m thin plane
at the origin. The plane is a static body because we do not set its mass
(equivalent to setting it to zero in Newton). The ball accelerates downwards
(negative y-direction) due to the force we apply in the callback.

Here is the :download:`code <./tut_bouncing_ball.cpp>`:

.. literalinclude:: tut_bouncing_ball.cpp
    :linenos:
    :language: c


Linux
-----
To compile, link, and run the program type:

.. code-block:: bash
    :linenos:

    $ g++ tut_bouncing_ball.cpp -o tut_bouncing_ball -I ../../coreLibrary_300/source/newton/ -L ../../build/lib/ -lNewton -lpthread -Wl,"-rpath=../../build/lib/"
    $ ./tut_bouncing_ball


Other Platform
--------------
Please submit pull request for instructions.


Result
------

The left figure graphs the ``y`` position of the ball and ground. As expected,
the ball drops, bounces off the ground, and eventually comes to rest. Note that
the rest position is 1.1m because the ball radius is 1m and the thickness of
the ground is 0.1m.

The right figure shows the ``sleeping`` attribute for each body as returned by
``NewtonBodyGetSleepState``. Newton puts the static body immediately to rest
because it is, well, static. The ball, on the other hand, stays active until it
has come to rest on the surface.

.. image:: img/tut_bouncing_ball_pos.png
   :width: 45%

.. image:: img/tut_bouncing_ball_sleeping.png
   :width: 45%



Questions/Problems
------------------
