Compiling And Linking A Trivial Program That Uses Newton
========================================================

The program will print some information about the engine and then quit.
Here is the :download:`code <./tut_compile.cpp>`:

.. literalinclude:: tut_compile.cpp
    :linenos:
    :language: c

Linux
-----
If you installed the Newton library (instead of merely compiling it inside your
build  directory) then you can compile, link, and run the program with these
commands:

.. code-block:: bash
    :linenos:

    $ g++ tut_compile.cpp -o tut_compile -lNewton -lpthread
    $ ./tut_compile

If you did *not* install the library, or want to use the version in your
``build`` directory, then you need to explicitly specify the header- and
library locations.

.. code-block:: bash
    :linenos:

    $ g++ tut_compile.cpp -o tut_compile -I ../../coreLibrary_300/source/newton/ -L ../../build/lib/ -lNewton -lpthread -Wl,"-rpath=../../build/lib/"
    $ ./tut_compile

If you are unfamiliar with these options, here is the breakdown: the ``-I
../../coreLibrary_300/source/newton/`` flag tells the compiler where to find
``Newton.h``. Similarly,  ``-L ../../build/lib/`` tells the linker where to look
for additional libraries (``libNewton.so`` in our case). The ``-lNewton`` flag
will link the program against the ``libNewton.so`` library we built in
:ref:`Installation`. Since Newton uses threads, we also need ``-lpthread``.
Finally, the purpose of the unwieldy ``-Wl,"-rpath=../../build/lib/"`` is
to bake the location of the Newton library into the executable, or otherwise
the runtime linker will probably not find it (just Google "gcc rpath" if you
want to know why the program will compile but not run without this option).


Other
-----
Please submit pull request for instructions.
