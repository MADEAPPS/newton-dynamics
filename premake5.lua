-- premake4.lua
workspace ("demosSandbox")
   configurations { "Debug", "Release" }

project "dgCore"
   location "sdk/projects"
   kind "StaticLib"
   language "C++"
   targetdir "sdk/dgCore"
-- targetdir "bin/%{cfg.buildcfg}"

   files { "sdk/dgCore/*.h", "sdk/dgCore/*.cpp" }

   filter "configurations:Debug"
      defines { "_DEBUG" }
      symbols "On"

   filter "configurations:Release"
      defines { "NDEBUG" }
      optimize "On"