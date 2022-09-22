building a cmake export

1- open Cmake
2- set all options off
3- set option NEWTON_BUILD_ANDROID_STUDIO on
4- selct a destination folder
5- click configure
6- click generate
6- open visual studion or your complier and build the project.
7- afte the buidl is completed, run install.
8- after cmake finish get find the cmake destination folder and grab folder <cmakeDst>/newtonAndroid
9- copy folder <cmakeDst>/newtonAndroid to the some directory in your android studio app/src
10- In android Studio righ click on file app/src/newtonAndroid/ndNewton/CMakeLists.txt
11- select Add C++ to module
12- select Link an existing cmakefile.txt or Android.mk to this module
13- brouse for that cmakefile.txt file on the file brouser
14- click ok.
15- wait until android parses the cmakefile.txt file.
16- now you should have newton integrated to your application. 