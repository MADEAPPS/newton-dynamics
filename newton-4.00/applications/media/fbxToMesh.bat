@echo off
setlocal

rem Define the starting directory. Use "." for the current directory.
set "target_dir=."

for /R "%target_dir%" %%f in (*.fbx) do (
    rem echo "processing %%f"
    fbxToMesh.exe %%f
    echo "processing %%f"
)
pause
endlocal