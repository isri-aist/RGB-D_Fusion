#pragma once
// UE
#include "CoreMinimal.h"
#include "Materials/Material.h"
#include "GameFramework/Actor.h"
#include "Engine/Texture2D.h"
#include "Components/DynamicMeshComponent.h"
#include "Components/Slider.h"
#include "Components/CheckBox.h"
#include "Modules/ModuleManager.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Containers/Queue.h"
#include "DynamicMeshActor.h"
#include <thread>
#include <atomic>

// K4A Driver For Microsoft Azure Kinect
#include </usr/include/k4a/k4a.hpp>

// rclUE
#include "ROS2Subscriber.h"
#include <Msgs/ROS2Img.h>

#include "ROS2SubscriberNode.generated.h"

UCLASS()
class RGBDIMMERSION_API AROS2SubscriberNode : public AActor
{
	GENERATED_BODY()
	
public:
    AROS2SubscriberNode();

	// Unreal Engine Functions
    virtual void BeginPlay() override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
    virtual void Tick(float DeltaSeconds) override;

	// Function called when a new image is received through a ROS2 Topic
    UFUNCTION()
    void RGBCallback(const UROS2GenericMsg* InMsg);
    UFUNCTION()
    void DepthCallback(const UROS2GenericMsg* InMsg);
    UFUNCTION()
    void IRCallback(const UROS2GenericMsg* InMsg);

	// Topics used to get the RGB, Depth and IR images
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    UROS2NodeComponent* Node = nullptr;
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FString RGBTopicName = TEXT("k4a/rgb/image_raw");
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FString DepthTopicName = TEXT("k4a/depth/image_raw");
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FString IRTopicName = TEXT("k4a/ir/image_raw");

	// Information about the Images
    UPROPERTY()
    int WidthTexture = 0;
    UPROPERTY()
    int HeightTexture = 0;
    UPROPERTY()
    int WidthDepth = 0;
    UPROPERTY()
    int HeightDepth = 0;
	// Textures
    UPROPERTY()
    UTexture2D* Texture;
    UPROPERTY()
    UTexture2D* Depth;
    UPROPERTY()
    UTexture2D* IR;
	
	// Values of the UI
	float LastBlendValue = 0;
	bool LastMultiView = false, LastRGBView = false, LastDepthView = false, LastIRView = false, LastDepthSimu = true;
	// UI Elements
	UPROPERTY()
	USlider* Slider = nullptr;
	UPROPERTY()
	UCheckBox* MultiView = nullptr;
	UPROPERTY()
	UCheckBox* RGBView = nullptr;
	UPROPERTY()
	UCheckBox* DepthView = nullptr;
	UPROPERTY()
	UCheckBox* IRView = nullptr;
	UPROPERTY()
	UCheckBox* DepthSimu = nullptr;
	UPROPERTY()
	UCheckBox* TpCenter = nullptr;

	// Modifiable element of the Dynamic mesh Blueprint
	FObjectProperty* TextureProp;
	FBoolProperty* IsDepthSimuEnabled;

	// Stores the Current User Pawn
	UPROPERTY()
	APawn* PlayerPawn;

	// Define the function used in the thread to fill depth
    UFUNCTION()
    void InpaintDepth();
	std::thread InpaintThread;
    std::atomic<bool> bStopThread{false};
	// Queues used to send information to and from the thread
    TQueue<TArray<uint8>, EQueueMode::Mpsc> MsgQueue;
    TQueue<TSharedPtr<TArray<uint8>>, EQueueMode::Mpsc> MsgQueue2;
    TQueue<TSharedPtr<TArray<uint8>>, EQueueMode::Mpsc> DataQueue;
    TQueue<TSharedPtr<TArray<uint8>>, EQueueMode::Mpsc> DepthQueue;
    TQueue<size_t, EQueueMode::Mpsc> TotalQueue;

	// Store the Dynamic Material and a reference Material used to change the Texture during Runtime
    UPROPERTY()
    UMaterialInstanceDynamic* DynamicMaterialInstance;
	UPROPERTY(EditAnywhere, Category = "Dynamic")
	UMaterialInterface* BaseMaterial;

	// Store the Dynamic Mesh Object
	UPROPERTY(EditAnywhere, Category = "ROS2")
	ADynamicMeshActor* DynamicMeshActor;
    UPROPERTY()
    UDynamicMeshComponent* DynamicMesh;

	// Name of the different modifiable parameters of the Dynamic Material
    FName TextureParameterName = FName("Tex");
    FName TextureParameterName2 = FName("Depth");
    FName TextureParameterName3 = FName("IR");
    FName BlendParameterName = FName("BlendValue");
    FName RGBParameterName = FName("RGBView");
    FName DepthParameterName = FName("DepthView");
    FName IRParameterName = FName("IRView");
	FName MultiViewParameterName = FName("MultiView");
	FName DepthSimuParameterName = FName("DepthSimu");
	
	// Store the information needed by the K4A SDK to perform the Color to Depth Transformation
    k4a::calibration calib;
    k4a::transformation transformation;

	// Raw data received directly from ROS2 Topics
    TArray<uint8> RawDataTexture, RawDataIR;
	// Full information of Images
    TArray<uint8> BGRADataTexture, BGRADataDepth;
    TArray<uint16> BGRADataIR;

	// Variables used to Update the Textures
    bool bPendingTextureUpdate = false;
    bool bPendingDepthUpdate = false;
    bool bPendingIRUpdate = false;
};
