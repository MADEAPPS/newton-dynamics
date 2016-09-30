Attach Custom Data
==================

Newton allows the user to attach a custom data structure to each body, as well as the world. The function to set/get that are straightforward to use, as the next example demonstrates.

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
