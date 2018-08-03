-- premake4.lua
newoption {
   trigger     = "dll",
   description = "build dll libraries"
}

newoption {
   trigger     = "staticMT",
   description = "build static libraries using MT system libraies"
}

newoption {
   trigger     = "staticMD",
   description = "build static libraries using MD system libraies"
}

local config = "SharedLib"
if _OPTIONS["staticMT"] then
   config = "StaticLib"
end

if _OPTIONS["staticMD"] then
   config = "StaticLib"
end


workspace ("build")
   configurations { "debug", "release" }

project "dgCore"
   location "projects"
   kind config
   language "C++"
   targetdir "../dgCore"

   files { "../dgCore/*.h", "../dgCore/*.cpp" }

   filter "configurations:debug"
      defines { "_DEBUG" }
      symbols "On"

   filter "configurations:release"
      defines { "NDEBUG" }
      optimize "On"