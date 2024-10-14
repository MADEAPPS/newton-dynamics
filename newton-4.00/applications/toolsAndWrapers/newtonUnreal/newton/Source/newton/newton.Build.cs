// Copyright Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;

public class newton : ModuleRules
{
	public newton(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;

        PrivateDefinitions.Add("_D_CORE_DLL");
        PrivateDefinitions.Add("_D_NEWTON_DLL");
        PrivateDefinitions.Add("_D_COLLISION_DLL");

        PublicIncludePaths.AddRange(
			new string[] {
				// ... add public include paths required here ...
			}
			);
				
		
		PrivateIncludePaths.AddRange(
			new string[] {
				// ... add other private include paths required here ...
				"ThirdParty/newtonLibrary/Public/dCore/",
                "ThirdParty/newtonLibrary/Public/dNewton/",
                "ThirdParty/newtonLibrary/Public/dCollision/",
                "ThirdParty/newtonLibrary/Public/thirdParty/",
                "ThirdParty/newtonLibrary/Public/dNewton/dJoints/",
                "ThirdParty/newtonLibrary/Public/dNewton/dModels/",
                "ThirdParty/newtonLibrary/Public/dNewton/dIkSolver/",
                "ThirdParty/newtonLibrary/Public/dNewton/dModels/dVehicle/"
            }
			);
			
		
		PublicDependencyModuleNames.AddRange(
			new string[]
			{
				"Core",
				"Slate",
				"Engine",
                "UnrealEd",
                "Projects",
				"SlateCore",
                "ToolMenus",
                "Landscape",
                "CoreUObject",
                "PhysicsCore",
                "GeometryCore",
                "EditorFramework",
				"GeometryFramework",

				// ... add other public dependencies that you statically link with here ...
				"newtonLibrary"
            }
            );
			
		
		PrivateDependencyModuleNames.AddRange(
			new string[]
			{
				// ... add private dependencies that you statically link with here ...	
			}
			);
		
		
		DynamicallyLoadedModuleNames.AddRange(
			new string[]
			{
				// ... add any modules that your module loads dynamically here ...
			}
			);
	}
}
