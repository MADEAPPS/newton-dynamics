![emscripten logo](https://raw.githubusercontent.com/MADEAPPS/newton-dynamics/master/applications/newtonLogo/newtonLogo.png)

Newton dynamics is a realistic, cross-platform physics simulation library. It can easily be integrated into games and game engines and provides top of it's class performance and simulation stability.
Ongoing developement and a permissive license makes Newton Dynamics a top choice for all kinds of projects from scientific projects to game engines.

* Links to **demos**, **tutorial**, **FAQ**, etc: <https://github.com/MADEAPPS/newton-dynamics/wiki>
* Main project page: <http://newtondynamics.com>
* Forums and public discussion: <http://newtondynamics.com/forum/index.php>

Compiling Newton
================

## Windows and OSX
Project files for Visual Studio and XCode are provided, mainly

* Newton Core: [coreLibrary_300/projects/](coreLibrary_300/projects/)
* packages: [packages/projects/](packages/projects/)
* demo sandbox: [applications/demosSandbox/projects/](applications/demosSandbox/projects/)

## Linux
There are Unix makefiles in the project folders mentioned above, but it is *highly* recommended to use CMake indstead.

Newton Core does not have any third party dependencies.

For building the demo sandbox the following packages need to be installed:
* wxWidgets 2.8 or 2.9
* Freetype
* OpenGL
* GLEW
* OpenAL
* TinyXML

All of these should be available on any major Linux distribution (with associated `-dev` or `-devel` packages).

Alternatively they can be found in the folder [packages/thirdParty/](packages/thirdParty/) and built from source.


License
=======
Newton Dynamics is licensed under the zlib open source license, with little if any practical difference between them.

See `LICENSE` for the full content of the licenses.

Authors
=======
Newton Dynamics is a project of Julio Jerez and Alain Suero. See `AUTHORS` for a full list of contributors.
