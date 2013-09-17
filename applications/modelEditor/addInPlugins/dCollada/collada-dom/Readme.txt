PLAYSTATION(R)3 COLLADA Package
            Copyright (C) 2008 Sony Computer Entertainment Inc.
                    All Rights Reserved.

==============================================================================
Contents of COLLADA Package 
==============================================================================

This package includes the COLLADA DOM and the COLLADA Viewer.  Everything in 
this package can be built to run on either PS3 or a Windows PC.  For additional 
information on tools and plugins that support COLLADA, or to get help from the
Internet community of COLLADA users, please visit www.collada.org  

The COLLADA DOM is a set of libraries for loading and saving COLLADA documents
that can contain 2D, 3D, physics and other types of content. It allows developers 
to create applications that can exchange COLLADA documents with commercial content 
creation tools such as Maya, Max or Softimage.  

The COLLADA Viewer is a general purpose 3D viewer for COLLADA documents.  It is
provided as sample code to show how to use the COLLADA DOM and to demonstrate
the kinds of content that can be stored in a COLLADA document.  The included
sample COLLADA documents can be loaded into the Viewer to demonstrate these
features.  The COLLADA Viewer and its sample documents replace all the 
hard coded COLLADA demos that were included with older PS3 releases.

<COLLADA_ROOT>\
 +--dom                     The DOM library for Parsing COLLADA Documents
 |  +--CodeGen
 |  +--external-libs        Open source libraries used by DOM
 |  +--include
 |  +--make
 |  +--projects             Project files for PS3, Win32 and Macintosh platforms
 |  +--src                  DOM source code
 |  +--test                 Source code for COLLADA DOM's automated test suite
 |  +--...
 |
 +--fx                      The COLLADA effect loader library for Cg shaders
 |  +--build
 |  +--include
 |  +--make
 |  +--projects             Project files for PS3, Win32 and Macintosh platforms
 |  +--src                  COLLADA effect loader source code
 |  +--...
 |
 +--License_Folder          Licenses for open source software included in COLLADA Viewer
 |  +--other
 |  +--license_e.txt
 |
 +--rt                      The Runtime Rendering library that uses OpenGL ES or PSGL
 |  +--build
 |  +--external-libs        Open source libraries used by rt module
 |  +--include
 |  +--make
 |  +--projects             Project files for PS3, Win32 and Macintosh platforms
 |  +--src                  COLLADA runtime rendering library source code
 |  +--...
 |
 +--viewer					
 |  +-bin                   The application binary that uses all software libraries above to make a COLLADA document viewer
 |     +-samples.zip        Compressed sample COLLADA documents used by COLLADA Viewer
 |     +-samples.doc        Description of sample COLLADA documents
 |
 |  +-make       
 |     +-common.mk
 |     +-rules.mk
 |     +-viewer.mk
 |
 |  +-projects
 |     +-linux              Source code for Linux COLLADA Viewer
 |     +-vc8-PS3            Visual Studio 2005 project files and platform-specific source code for PS3 COLLADA Viewer
 |     +-vc8-Win            Visual Studio 2005 project files and platform-specific source code for Windows native COLLADA Viewer
 |     +-vc9-PS3            Visual Studio 2008 project files and platform-specific source code for PS3 COLLADA Viewer
 |     +-vc9-Win            Visual Studio 2008 project files and platform-specific source code for Windows native COLLADA Viewer
 |     +-xcode              Macintosh Xcode project files and source code for Macintosh COLLADA Viewer
 |
 |  +--...
 |
 +--Makefile                Makefile (for Linux and MSYS) to build COLLADA Viewer and the necessary libraries
 +--Makefile.linux          Makefile to build Linux COLLADA Viewer and the necessary libraries
 +--Readme.txt              This file



=============================
Requirements to build for PS3
=============================

[Windows host/MSYS]
MinGW (Minimalist GNU for Windows)
MSYS with GNU make version 3.81
PS3 SDK toolchain
FW sample framework (prebuilt)


[Windows host/Visual Studio]
Visual Studio 2005 or Visual Studio 2008
PS3 SDK toolchain
ProDG Visual Studio integration
FW sample framework (prebuilt)

[Linux]
GNU make version 3.81
PS3 SDK toolchain
FW sample framework (built)


=================================
Requirements to build for Windows
=================================

[Windows native]
Visual Studio 2005 or Visual Studio 2008
Cg 2.0 Toolkit


===============================
Requirements to build for Linux
===============================

OpenGL and GLUT libraries
Cg 2.0 Toolkit


===================================
Requirements to build for Macintosh
===================================

Xcode 3.11
Cg 2.0 Toolkit




==========================================================================
Building the COLLADA package and running the COLLADA Viewer sample for PS3
==========================================================================

These instructions will build the entire contents of the COLLADA package including
libraries and samples for the PS3.  To build for the PS3, the PS3 SDK must 
be installed.  The build instructions include running the COLLADA Viewer
which can be used to view any of the included sample COLLADA documents.  


[Windows host/MSYS]
1) Execute make utility at root level of package by typing 'make'
   viewer.self will be in <COLLADA_ROOT>\viewer\bin for release build (conf=release, this is the default build)
   viewer.self will be in <COLLADA_ROOT>\viewer\bin for debug build (conf=debug)
2) Unzip samples.zip, extract its contents to <COLLADA_ROOT>\viewer\bin\
3) Use ProDG Target Manager or Debugger to load viewer.self 
   Optionally, specify a COLLADA document (cage.dae) as command line argument when loading viewer executable


[Windows host/Visual Studio]
Project files for Visual C++ 8 (VS 2005) and Visual C++ 9 (VS 2008) are both provided. If you are using Visual C++ 9,
substitute "vc9" for "vc8" for the following instructions.
1) Open Visual Studio Solution at <COLLADA_ROOT>\viewer\projects\vc8-PS3\viewer.sln
2) Select "PS3 debug" or "PS3 release" configuration to build PS3 target
3) Unzip samples.zip, extract its contents to <COLLADA_ROOT>\viewer\bin\
4) Use ProDG Target Manager or Debugger to load viewer.ppu.self (release) or viewer-d.ppu.self in \viewer\bin
   Optionally, specify a COLLADA document (cage.dae) as command line argument when loading viewer executable

[Linux]
1) Execute make utility at root level of package by typing 'make'
   viewer.self will be in <COLLADA_ROOT>\viewer\bin for release build (conf=release, this is the default build)
   viewer.self will be in <COLLADA_ROOT>\viewer\bin for debug build (conf=debug)
2) Unzip samples.zip, extract its contents to <COLLADA_ROOT>\viewer\bin\
3) Change directory to where the COLLADA Viewer executable is, type the following commands to load it, replace
   the IP address with your PS3 IP address.
   
	> bedbg -prepare
	> dtpon -d 10.98.12.34
	> bedbg -nodebug viewer.self
	
	Type the following commands to terminate COLLADA Viewer
	> bedbg -T
	> dtpoff -d 10.98.12.34


==============================================================================
Building the COLLADA package and running the COLLADA Viewer sample for Windows
==============================================================================

These instructions will build the entire contents of the COLLADA package including
libraries and samples for Windows.  All the libraries needed to build for Windows
are included in this package or come with Visual Studio.  The instructions include 
running the COLLADA Viewer which can be used to view any of the included sample 
COLLADA documents.

Project files for Visual C++ 8 (VS 2005) and Visual C++ 9 (VS 2008) are both provided. If you are using Visual C++ 9,
substitute "vc9" for "vc8" for the following instructions.

1) Open Visual Studio Solution at <COLLADA_ROOT>\viewer\projects\vc8-Win\viewer.sln
2) Select "Debug" or "Release" configuration to build Windows executable
3) viewer.exe (release) or viewer-d.exe(debug) will be in <COLLADA_ROOT>\viewer\bin
4) Unzip samples.zip, extract its contents to <COLLADA_ROOT>\viewer\bin\
5) Open up a DOS Shell window, change directory to <COLLADA_ROOT>\viewer\bin, type "viewer.exe duck.dae" to view COLLADA document "duck.dae"


============================================================================
Building the COLLADA package and running the COLLADA Viewer sample for Linux
============================================================================

These instructions build the entire contents of the COLLADA package including libraries and samples for Linux PC. 

1) Type 'make -f Makefile.linux' at root level of package - <COLLADA_ROOT>.
   COLLADA Viewer 'viewer' will be created in <COLLADA_ROOT>/viewer/build/linux-1.4 for release build (conf=release, this is the default build)
   'viewer' will be created in <COLLADA_ROOT>/viewer/build/linux-1.4-d for debug build 'make -f Makefile.linux conf=debug'
2) Unzip samples.zip, extract its contents to <COLLADA_ROOT>/viewer/build/linux-1.4(-d)
3) Change directory to where 'viewer' executable is, type './viewer cage.dae' to load and render COLLADA document cage.dae



================================================================================
Building the COLLADA package and running the COLLADA Viewer sample for Macintosh
================================================================================

These instructions build the entire contents of the COLLADA package including libraries and samples for Macintosh PC

1) Open Xcode project file at <COLLADA_ROOT>/viewer/projects/xcode/viewer.xcodeproj
2) Select "Debug" or "Release" configuration to build Macintosh executable
3) The release or debug 'viewer' executable will be created in <COLLADA_ROOT>/viewer/bin
4) Unzip samples.zip, extract its contents to <COLLADA_ROOT>/viewer/bin/
5) Click 'Run' from Xcode menu bar to load default COLLADA document 'cage.dae'
6) Optionally, add command line argument to Xcode Executable 'viewer' info to specify different COLLADA document to load




===========================
COLLADA Viewer button usage
===========================
[PS3]

Up          Navigate the document browser upward
Down        Navigate the document browser downward
Cross       Load the document browser selected COLLADA document
Select      Next camera view
L1          Zoom out, move camera away from the focus point
L2          Zoom in, move camera closer to the focus point
L3          Toggle Browser thumbnail display
R3          Toggle wireframe display
Left analog stick	Move camera and focus point up, down, left, right
Right analog stick 	Rotate camera up, down, left right


[Windows native]

Keybroad:		  
Tab         Next camera
M           Increase key press and mouse responsiveness by a factor of 25%
N           Decrease key press and mouse responsiveness by a factor of 25%
            Note: M & N key presses affect the responsiveness of W/S/A/D/X/Space keys, left mouse click, right mouse click and mouse wheel
            
Q           Toggle Wireframe
K           Show node hierarchy with lines
L           Toggle Lighting mode
P           Toggle animation
F1          Toggle full screen

W           Move camera closer to the focus point, zoom in
S           Move camera away from the focus point, zoom out
A           Move camera and focus point to the left
D           Move camera and focus point to the right
Space       Move camera and focus point up
X           Move camera and focus point down
F           Change triangle/quad culling setting; rotate between frontface-culling/no-culling/backface-culling

Mouse:
Wheel Up            Move camera closer to the focus point, zoom in
Wheel Down          Move camera away from the focus point, zoom out
Left Click Move     Rotate camera up, down, left and right
Right Click Move    Move camera and focus point up, down, left, right
Middle Click        Next camera


[Linux/Macintosh]

Keybroad:		  
Tab         Next camera
M           Increase key press and mouse responsiveness by a factor of 25%
N           Decrease key press and mouse responsiveness by a factor of 25%
            Note: M & N key presses affect the responsiveness of W/S/A/D/X/Space keys, left mouse click, right mouse click 
            
Q           Toggle Wireframe
K           Show node hierarchy with lines
L           Toggle Lighting mode
P           Toggle animation
F1          Toggle full screen

W           Move camera closer to the focus point, zoom in
S           Move camera away from the focus point, zoom out
A           Move camera and focus point to the left
D           Move camera and focus point to the right
Space       Move camera and focus point up
X           Move camera and focus point down
F           Change triangle/quad culling setting; rotate between frontface-culling/no-culling/backface-culling

Mouse:
Left Click Move     Rotate camera up, down, left and right
Right Click Move    Move camera and focus point up, down, left, right
Middle Click        Next camera



============
Known issues
============

- For PS3 debug build, when unloading COLLADA document with large number of nodes, such as demo.dae and dominos.dae,
  a debug exception is raised due to stack overflow.
  
  The COLLADA Viewer uses Cell SDK framework, which sets the main PPU thread priority and stack size in its code base.
  Currently there is no way to override this stack size setting without generating a linker warning on Windows host or 
  an error on Linux host.  See below:

  linker warning "L0019: symbol '__sys_process_param' multiply defined" is generated.
  
  To get around this problem, one can modify the Cell SDK framwork code in the following way:
  
  In <SDKROOT>\samples\fw\src\cell\FWCellMain.cpp, modify the line from 
	SYS_PROCESS_PARAM( 1001, 0x10000 );		// priority = 1001, stack size = 64K	
	
  to
  
	SYS_PROCESS_PARAM( 1001, 0x18000 );		// priority = 1001, increase stack size to 96K, or other setting to suit your need

