Compiling And Linking A Trivial Program That Uses Newton
========================================================

Purpose: show how to compile a Newton simulation.

The simulation creates a Newton world, steps it sixty times, and then destroys
the world again. Literally nothing will happen since the world is empty. Here
is the :download:`code <./tut0.cpp>`:

.. literalinclude:: tut0.cpp
    :linenos:
    :language: c

Linux
-----
If you installed the Newton library into your system the you can compile, link,
and run the program with these commands:

.. code-block:: bash
    :linenos:

    $ g++ tut0.cpp -o tut0 -lNewton -lpthread
    $ ./tut0

If you did _not_ install the library and want to use the version from your
``source`` and ``build`` directory, then you need a longer command:

.. code-block:: bash
    :linenos:

    $ g++ tut0.cpp -o tut0 -I ../../coreLibrary_300/source/newton/ -L ../../build/lib/ -lNewton -lpthread -Wl,"-rpath=../../build/lib/"
    $ ./tut0

This may look intimidating but is straightforward. The ``-I
../../coreLibrary_300/source/newton/`` flag tells the compiler where to find
``Newton.h``. Similarly,  ``-L ../../build/lib/`` tells the linker where to look
for additional libraries. The next flag (``-lNewton``) tells the linker to link
our program against the ``libNewton.so`` library we built in :ref:`Installation`.
Newton uses threads, thus we linke it to ``-lpthread``. Finally, the purpose of
``-Wl,"-rpath=../../build/lib/"`` is to bake the location of the Newton library
into the executable, or otherwise the runtime linker will probably not find it.


Other
-----
Please submit pull request for instructions.
