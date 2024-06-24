# Copyright (c) <2014-2017> <Newton Game Dynamics>
#
# This software is provided 'as-is', without any express or implied
# warranty. In no event will the authors be held liable for any damages
# arising from the use of this software.
#
# Permission is granted to anyone to use this software for any purpose,
# including commercial applications, and to alter it and redistribute it
# freely.

cmake_minimum_required(VERSION 3.9.0 FATAL_ERROR)

set (projectName "ndBrainVulkan")
message (${projectName})

include_directories(.)
include_directories(../dCore)

if(NEWTON_ENABLE_VULKAN_SDK)
	include_directories ($ENV{VULKAN_SDK}/Include)
endif()


add_definitions(-D_D_BRAIN_VULKAN_DLL_EXPORT)

file(GLOB CPP_SOURCE *.cpp *.h)
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/" FILES ${CPP_SOURCE})

add_library(${projectName} STATIC ${CPP_SOURCE})

if (MSVC)
	set_target_properties(${projectName} PROPERTIES COMPILE_FLAGS "/YundBrainVulkanStdafx.h")
	set_source_files_properties(ndBrainVulkanStdafx.cpp PROPERTIES COMPILE_FLAGS "/YcndBrainVulkanStdafx.h")
endif()

if (NEWTON_ENABLE_VULKAN_SDK)
	set(BRAIN_SHADER_FOLDER "${CMAKE_SOURCE_DIR}/sdk/dMedia/dbrain")
	file (GLOB BRAIN_SHADERS ${BRAIN_SHADER_FOLDER}/source/*.comp)

	#execute_process (COMMAND $ENV{VULKAN_SDK}/Bin/glslang.exe -h)

	foreach(file IN LISTS BRAIN_SHADERS)
		#message (${file})
		get_filename_component(shaderName ${file} NAME_WE)
		get_filename_component(dotShaderExt ${file} EXT)
		string(REPLACE "\." "-" shaderExt ${dotShaderExt})

		file(TIMESTAMP ${BRAIN_SHADER_FOLDER}/${shaderName}${shaderExt}.spv dstDate)
		if(dstDate)
			file(TIMESTAMP ${file} srcDate)
			#message (${srcDate} " " ${dstDate})
			if(${srcDate} STRGREATER ${dstDate})
				message (${dstDate} " " ${BRAIN_SHADER_FOLDER}/${shaderName}${shaderExt}.spv)
				#execute_process (COMMAND $ENV{VULKAN_SDK}/Bin/glslang --target-env spirv1.3 -V ${file} -o ${BRAIN_SHADER_FOLDER}/${shaderName}${shaderExt}.spv)
				#execute_process (COMMAND $ENV{VULKAN_SDK}/Bin/spirv-opt -O ${BRAIN_SHADER_FOLDER}/${shaderName}${shaderExt}.spv -o ${BRAIN_SHADER_FOLDER}/${shaderName}${shaderExt}xxx.spv)
				#execute_process (COMMAND $ENV{VULKAN_SDK}/Bin/spirv-dis ${BRAIN_SHADER_FOLDER}/${shaderName}${shaderExt}.spv  -o ${BRAIN_SHADER_FOLDER}/${shaderName}${shaderExt}.asm)
				#execute_process (COMMAND $ENV{VULKAN_SDK}/Bin/spirv-dis ${BRAIN_SHADER_FOLDER}/${shaderName}${shaderExt}xxx.spv -o ${BRAIN_SHADER_FOLDER}/${shaderName}${shaderExt}xxx.asm)

				execute_process (COMMAND $ENV{VULKAN_SDK}/Bin/glslang --target-env spirv1.3 -V ${file} -o ${BRAIN_SHADER_FOLDER}/${shaderName}-temp.spv)
				execute_process (COMMAND $ENV{VULKAN_SDK}/Bin/spirv-opt -O ${BRAIN_SHADER_FOLDER}/${shaderName}-temp.spv -o ${BRAIN_SHADER_FOLDER}/${shaderName}${shaderExt}.spv)
				execute_process (COMMAND $ENV{VULKAN_SDK}/Bin/spirv-dis ${BRAIN_SHADER_FOLDER}/${shaderName}${shaderExt}.spv -o ${BRAIN_SHADER_FOLDER}/${shaderName}${shaderExt}.asm)
				execute_process (COMMAND $ENV{VULKAN_SDK}/Bin/spirv-dis ${BRAIN_SHADER_FOLDER}/${shaderName}-temp.spv -o ${BRAIN_SHADER_FOLDER}/${shaderName}-temp.asm)
			endif()
		else()
			message (${BRAIN_SHADER_FOLDER}/${shaderName}${shaderExt}.spv)
			#execute_process (COMMAND $ENV{VULKAN_SDK}/Bin/glslang --target-env spirv1.3 -V ${file} -o ${BRAIN_SHADER_FOLDER}/${shaderName}${shaderExt}.spv)
			execute_process (COMMAND $ENV{VULKAN_SDK}/Bin/glslang --target-env spirv1.3 -V ${file} -o ${BRAIN_SHADER_FOLDER}/${shaderName}-temp.spv)
			execute_process (COMMAND $ENV{VULKAN_SDK}/Bin/spirv-opt -O ${BRAIN_SHADER_FOLDER}/${shaderName}-temp.spv -o ${BRAIN_SHADER_FOLDER}/${shaderName}${shaderExt}.spv)
			execute_process (COMMAND $ENV{VULKAN_SDK}/Bin/spirv-dis ${BRAIN_SHADER_FOLDER}/${shaderName}${shaderExt}.spv -o ${BRAIN_SHADER_FOLDER}/${shaderName}${shaderExt}.asm)
			execute_process (COMMAND $ENV{VULKAN_SDK}/Bin/spirv-dis ${BRAIN_SHADER_FOLDER}/${shaderName}-temp.spv -o ${BRAIN_SHADER_FOLDER}/${shaderName}-temp.asm)
		endif()
	endforeach()

	install(DIRECTORY "${CMAKE_SOURCE_DIR}/sdk/dMedia" DESTINATION "${PROJECT_BINARY_DIR}/sdk")
endif()

if (MSVC)
	set_target_properties(${projectName} PROPERTIES FOLDER "deepLearning")
endif()

