// Fill out your copyright notice in the Description page of Project Settings.

#include "Math/UnrealMathUtility.h"
#include "Camera/CameraComponent.h"
#include "ROS2PublisherNode.h"

#include "MotionControllerComponent.h"

// Sets default values
AROS2PublisherNode::AROS2PublisherNode()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

	Node = CreateDefaultSubobject<UROS2NodeComponent>(TEXT("ROS2NodeComponent"));
	Node->Name = TEXT("unreal_publisher_node");
}

// Called when the game starts or when spawned
void AROS2PublisherNode::BeginPlay()
{
	Super::BeginPlay();
	Node->Init();

	ROS2_CREATE_LOOP_PUBLISHER_WITH_QOS(Node, this, McRtcTopicName, UROS2Publisher::StaticClass(), UROS2StringMsg::StaticClass(), 1000, &AROS2PublisherNode::PublishJointState, UROS2QoS::Default, LoopPublisher);

	PlayerPawn = UGameplayStatics::GetPlayerPawn(GetWorld(), 0);
}

// Called every frame
void AROS2PublisherNode::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

}


void AROS2PublisherNode::PublishJointState(UROS2GenericMsg* InMessage) {
	FROSString msg;

	if (PlayerPawn) {
		// Get the position of the VR Headset in the Scene
		TArray<UCameraComponent*> CameraComponents;
		TArray<UMotionControllerComponent*> MotionComponents;
		PlayerPawn->GetComponents<UCameraComponent>(CameraComponents);
		PlayerPawn->GetComponents<UMotionControllerComponent>(MotionComponents);
             
		// Compute the Mesh Position Minus the Camera Position to align the two elements
		for (UCameraComponent* CameraComp : CameraComponents) {
			for (UMotionControllerComponent* MotionComp : MotionComponents)
			{
				if (MotionComp->GetName() == "MotionControllerRight")
				{
					FRotator rot = CameraComp->GetRelativeRotation();
					FVector HandPos = MotionComp->GetComponentLocation();
					FRotator HandRot = MotionComp->GetComponentRotation();

					HandPos -= CameraComp->GetComponentLocation();

					float CameraRoll  =  rot.Roll  * PI / 180.0f, CameraPitch = -rot.Pitch * PI / 180.0f, CameraYaw   = -rot.Yaw   * PI / 180.0f;
					float HandRoll  =  HandRot.Roll  * PI / 180.0f, HandPitch = -HandRot.Pitch * PI / 180.0f, HandYaw   = -HandRot.Yaw   * PI / 180.0f;
					float HandX  =  HandPos.X, HandY  =  HandPos.Y, HandZ  =  HandPos.Z;

					msg.Data = FString::Printf(
						TEXT("{\"NECK_Y\": %f, \"NECK_P\": %f, \"NECK_R\": %f, ")
						TEXT("\"RHAND_POS_X\": %f, \"RHAND_POS_Y\": %f, \"RHAND_POS_Z\": %f, ")
						TEXT("\"RHAND_ROLL\": %f, \"RHAND_PITCH\": %f, \"RHAND_YAW\": %f}"),
						CameraYaw, CameraPitch, CameraRoll,
						HandX, HandY, HandZ,
						HandRoll, HandPitch, HandYaw
					);
					break;
				}
			}
		}
	}
	
	CastChecked<UROS2StringMsg>(InMessage)->SetMsg(msg);
}
