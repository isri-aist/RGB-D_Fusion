#pragma once

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "UDynamicMesh.h"
#include "Engine/Texture2D.h"
#include "MeshUtils.generated.h"

/**
 * 
 */
UCLASS()
class RGBDIMMERSION_API UMeshUtils : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()
	
public:
	UFUNCTION(BlueprintCallable, Category = "Mesh|Subdivision")
	static UDynamicMesh* SubdivideDynamicMesh(UDynamicMesh* DynamicMesh);

	UFUNCTION(BlueprintCallable, Category="Mesh|Displacement")
	static UDynamicMesh* DisplaceMeshFromTexture(
		UDynamicMesh* DynMesh,
		UTexture2D* DisplacementTexture,
		float Magnitude = 10.0f,
		float NeutralGray = 0.5f
	);
};