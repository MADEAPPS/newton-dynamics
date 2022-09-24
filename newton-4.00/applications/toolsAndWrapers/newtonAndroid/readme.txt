The anodirs studio cmake scrip will crate a blanck aroid studion app 
in the selected folder under directory ../androidApp 

1- open Cmake
3- set option NEWTON_BUILD_ANDROID_STUDIO on
4- select a destination folder
5- click configure
6- click generate

now you can open android studio and open project ../androidApp and you 
should have pure blank androids studio project with java classes 
wrapping the newton engine.

for and aready existing android studio application, the user can manually copy folders
..\androidApp\app\src\main\cpp ->to you repective cpp folder in your app 
..\androidApp\app\src\main\java\com\newton ->to you repective java folder in your app 

the last step is in android studio explorer, 
select file  ..\androidApp\app\src\main\cpp\CMakeLists.txt

right click and from the dropdown menu, select add C++ to module
them in the dialog select "Link an existing CmakeList.txt or Android.mk to this module"
the in the list box browse for the path to the CMakeLists.txt and click ok.

after that the application should be ready to go.


