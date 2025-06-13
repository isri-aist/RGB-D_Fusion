#pragma once

// UE
#include "CoreMinimal.h"

// ROS
#include "std_msgs/msg/string.h"

// rclUE
#include "Msgs/ROS2GenericMsg.h"
#include "rclcUtilities.h"

#include "ROS2String.generated.h"

USTRUCT(Blueprintable)
struct RCLUE_API FROSString
{
    GENERATED_BODY()

public:
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FString Data;
    
    FROSString()
    {
    }

    void SetFromROS2(const std_msgs__msg__String& in_ros_data)
    {
        Data = UROS2Utils::StringROSToUE<rosidl_runtime_c__String>(in_ros_data.data);
    }

    void SetROS2(std_msgs__msg__String& out_ros_data) const
    {
        UROS2Utils::StringUEToROS(Data, out_ros_data.data);
    }
};

UCLASS()
class RCLUE_API UROS2StringMsg : public UROS2GenericMsg
{
    GENERATED_BODY()

public:
    virtual void Init() override;
    virtual void Fini() override;

    virtual const rosidl_message_type_support_t* GetTypeSupport() const override;

    UFUNCTION(BlueprintCallable)
    void SetMsg(const FROSString& Input);

    UFUNCTION(BlueprintCallable)
    void GetMsg(FROSString& Output) const;

    virtual void* Get() override;

private:
    virtual FString MsgToString() const override;

    std_msgs__msg__String string_msg;
};

