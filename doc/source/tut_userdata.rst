Attach Custom Data To A Newton Body
===================================

Newton provides ``NewtonBodySetUserData`` and ``NewtonBodySetUserData`` to set
and retrieve custom user data, respectively. Their usage is straightforward as
the next example illustrates.

Here is the :download:`code <./tut_userdata.cpp>`:

.. literalinclude:: tut_userdata.cpp
    :linenos:
    :language: c


Linux
-----
To compile, link, and run the program type:

.. code-block:: bash
    :linenos:

    $ g++ tut_userdata.cpp -o tut_userdata -I ../../coreLibrary_300/source/newton/ -L ../../build/lib/ -lNewton -lpthread -Wl,"-rpath=../../build/lib/"
    $ ./tut_userdata


Other
-----
Please submit pull request for instructions.


Questions/Problems
------------------
