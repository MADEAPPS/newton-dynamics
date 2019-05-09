
Vulkan Runtime
==============

This directory contains the files required for distributing the Windows Vulkan
Runtime. This includes an installer, `VulkanRT-<version>-Installer.exe` and both
32-bit and 64-bit binaries for vulkan-1.dll and vulkaninfo.exe. A Vulkan driver
or application may include either the installer or the files in its installation
to ensure that Vulkan is properly installed on a machine. If the runtime
installer is used, no extra work needs to be done by the driver or application.
If the binary files are installed manually, the driver or application must
adhere to the behavior outlined in this document, to ensure that the
installation will not conflict with other drivers or applications. This behavior
is intended to be implemented with an INF file, allowing easy installation from
driver packages.


Installation Process
--------------------

The runtime is provided as both an installer and as individual files which may
be packaged into INF installer. This is done to provide both an easy executable
installer, and to allow drivers to easily install the Vulkan runtime from an INF
file. As such, there are two methods of installing the Vulkan runtime. The
behavior of both installation methods is intended to match exactly, allowing the
two methods to be used interchangeably.

### Runtime Installer

The runtime installer copies four files into `System32`, and on a 64-bit system
copies four more files into `SysWOW64`. The files are:

- `vulkan-1.dll` and `vulkan-1-999-0-0-0.dll` (which are identical)
- `vulkaninfo.exe` and `vulkaninfo-1-999-0-0-0.exe` (which are identical)

These files are only installed if there is not a newer version of the file
already in the installation path. On a 64-bit system, the runtime installer
copies the 64-bit versions of the files given above to System32 and copies the
32-bit versions to SysWOW64. On a 32-bit system, the installer copies the 32-bit
files to System32 and ignores the 64-bit files.

### INF Installation

To install the Vulkan runtime from a driver INF file, a driver should perform
the following tasks on a 64-bit OS:

- Use `CopyFiles` with `COPYFLG_NO_VERSION_DIALOG` to copy `RunTimeInstaller\x64\vulkan-1.dll` to both `System32\vulkan-1.dll` and `System32\vulkan-1-999-0-0-0.dll`.
- Use `CopyFiles` with `COPYFLG_NO_VERSION_DIALOG` to copy `RunTimeInstaller\x64\vulkaninfo.exe` to both `System32\vulkaninfo.exe` and `System32\vulkaninfo-1-999-0-0-0.exe`.
- Use `CopyFiles` with `COPYFLG_NO_VERSION_DIALOG` to copy `RunTimeInstaller\x86\vulkan-1.dll` to both `SysWOW64\vulkan-1.dll` and `SysWOW64\vulkan-1-999-0-0-0.dll`.
- Use `CopyFiles` with `COPYFLG_NO_VERSION_DIALOG` to copy `RunTimeInstaller\x86\vulkaninfo.exe` to both `SysWOW64\vulkaninfo.exe` and `SysWOW64\vulkaninfo-1-999-0-0-0.exe`.

On a 32-bit OS, the driver INF should perform the following actions:

- Use `CopyFiles` with `COPYFLG_NO_VERSION_DIALOG` to copy `RunTimeInstaller\x86\vulkan-1.dll` to both `System32\vulkan-1.dll` and `System32\vulkan-1-999-0-0-0.dll`.
- Use `CopyFiles` with `COPYFLG_NO_VERSION_DIALOG` to copy `RuntimeInstaller\x86\vulkaninfo.exe` to both `System32\vulkaninfo.exe` and `System32\vulkaninfo-1-999-0-0-0.exe`.

In addition to this, driver INF installations should include the license files
in this directory, `LICENSE.txt` and `VULKANRT_LICENSE.rtf`. These files may
be installed to any directory of the driver's choosing.


Uninstallation
-------------

Neither the bundled executable installer, nor INF installations are allowed to
remove the files that were installed. As such, there is no uninstallation except
by manually deleting these files. If an uninstaller were to delete these files,
Vulkan would stop working on any system that has multiple Vulkan drivers
installed. Currently, there is no known method to solve this conflict without
violating the restrictions imposed by Windows on Universal Windows Drivers.

A driver uninstaller is allowed (though not required) to remove the license
files, `LICENSE.txt` and `VULKANRT_LICENSE.rtf` on uninstallation.


Old Runtimes
------------

In the past, the Vulkan runtime executable was shipped by all driver
installations. However, this installer violated the requirement imposed upon
newer driver installers by the requirements of Universal Windows Drivers.
Furthermore, the old runtime installer was overly complicated, as it attempted
to keep versioned copies of old runtimes so that it could downgrade systems
during an uninstallation.

While the old runtime logic has been retired, it is essential that new
installations preserve compatibility with old runtimes. This is the purpose of
the files with 999 in the filename. These files appear to old runtime installers
as Vulkan version 999. As such, they are always treated as the latest version,
which means that old runtime installers will always update vulkan-1.dll and
vulkaninfo.exe by overwriting these files with the 999 file. This is why the
999 files are required to be identical.

Furthermore, while `vulkan-1.dll` is required for a Vulkan application to run,
`vulkaninfo.exe` is not. However, due to the limitations of the old runtime
installer, `vulkaninfo.exe` must be installed as well. Old runtimes check for
the latest version of every file. If they do not match, the runtime installer
will fail and could end up downgrading the system to an older version of Vulkan.
As such, it is important that drivers install `vulkaninfo.exe` to maintain
backwards compatibility.
