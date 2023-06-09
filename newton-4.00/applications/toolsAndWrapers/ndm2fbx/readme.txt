This tool convert bvh and newton propieratry ndm files to autodesk FBX.

unfortunatlly the autodesk fbx sdk, is only free to use but not reditributable
therefore in order to build the tool, you most download a sdk version from autodesk.
At the time of this build that url is: 
https://www.autodesk.com/developer-network/platform-technologies/fbx-sdk-2020-0

after downloading and installing the SDK, you also need to create an user enviroment 
variable to point to the instalation folder FBX_SDK=[sdk_path]
for example:

#FBX_SDK=C:\Program Files\Autodesk\FBX\FBX SDK\2020.0.1/
FBX_SDK=C:\Program Files\Autodesk\FBX\FBX SDK\2020.3.2/

-open cmake and open the source folder
-select a build folder.
-click build
-open visual studio
-compile the tool