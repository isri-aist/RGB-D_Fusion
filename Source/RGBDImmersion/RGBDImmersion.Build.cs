// Fill out your copyright notice in the Description page of Project Settings.

using UnrealBuildTool;

public class RGBDImmersion : ModuleRules
{
	public RGBDImmersion(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
		CppStandard = CppStandardVersion.Cpp17;

        	PublicDependencyModuleNames.AddRange(new string[] { "Core", "CoreUObject", "Engine", "InputCore", "rclUE", "UMG", "RHI", "RenderCore", "zlib", "LibJpegTurbo", "libtiff", "OpenCV", "OpenCVHelper", "GeometryFramework" });
        	
        	PublicIncludePaths.Add("/home/raphael/Downloads/Linux_Unreal_Engine_5.1.0/Engine/Plugins/Runtime/OpenCV/Source/ThirdParty/OpenCV/include");
        	PublicIncludePaths.Add("/usr/include");
	      	PublicIncludePaths.Add("/usr/lib/x86_64-linux-gnu/libk4a.so");
	      	
	      	PublicAdditionalLibraries.Add("/usr/lib/x86_64-linux-gnu/libk4a.so");

		PrivateDependencyModuleNames.AddRange(new string[] {  });
	}
}
