@echo off
setlocal

rem Define the starting directory. Use "." for the current directory.
set "target_dir=."

for /R "%target_dir%" %%f in (*.fbx) do (
    rem echo "processing %%f"
    fbx2ndMesh.exe %%f
    echo "processing %%f"
)

endlocal