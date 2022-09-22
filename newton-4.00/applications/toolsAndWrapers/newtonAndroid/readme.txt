building a cmake export

1- open Cmake
2- set all options off
3- set option NEWTON_BUILD_NEWTON_ANDROID on
4- selct a destination folder
5- click configure
6- click generate
7- after cmake finish get find the cmake destination foller and grab folder <cmakeDst>/newtonAndroid
8- copy folder <cmakeDst>/newtonAndroid to the some directory in your android studio app/src
9- In android Studio righ click on file app/src/newtonAndroid/ndNewton/CMakeLists.txt
10- select Add C++ to module
11- select Link an existing cmakefile.txt or Android.mk to this module
12- brouse for that cmakefile.txt file on the file brouser
13- click ok.
14- wait until android parses the cmakefile.txt file.
15- now you should have newton integrated to your application. 