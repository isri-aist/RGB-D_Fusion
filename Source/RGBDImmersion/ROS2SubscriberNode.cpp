#include "ROS2SubscriberNode.h"

#include "Engine/Texture2D.h"
#include "IImageWrapper.h"
#include "IImageWrapperModule.h"
#include "Modules/ModuleManager.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Kismet/GameplayStatics.h"
#include "UObject/UnrealType.h"
#include <fstream>
#include <cstdlib>

#include "HeadMountedDisplayTypes.h"
#include "Components/WidgetComponent.h"
#include "tiffio.h"
#include "Blueprint/UserWidget.h"
#include "Camera/CameraComponent.h"
#include "GameFramework/SpringArmComponent.h"

// OpenCV
#ifdef check
#undef check
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/photo.hpp>
#endif

/*----------------------------------------------------------------*/
//			 			        Useful Functions 				            //
/*----------------------------------------------------------------*/
void AROS2SubscriberNode::InpaintDepth() {
   while (!bStopThread.load())
   {
      if (!MsgQueue.IsEmpty())
      {
         std::unique_lock<std::mutex> lock(QueueMutex);
         QueueCV.wait(lock, [this]() {
             return !MsgQueue.IsEmpty() || bStopThread;
         });

         if (bStopThread) break;
         
         // Acquire ROS2 Message
         TArray<uint8> Msg;
         MsgQueue.Dequeue(Msg);
         MsgQueue.Empty();
         lock.unlock();
		if (Msg.Num() < HeightDepth * WidthDepth * sizeof(float)) {
    		UE_LOG(LogTemp, Error, TEXT("Taille de message incorrecte pour profondeur"));
    		continue;
		}
         cv::Mat a(HeightDepth, WidthDepth, CV_32FC1, Msg.GetData()); // Create custom OpenCV Material

         cv::Mat inpainted, output_inv;

         // Define parameters used after
         const cv::Mat& kernel_far = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));
         const cv::Mat& kernel_med = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(5, 5));
         const cv::Mat& kernel_near = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(7, 7));
          
         cv::Mat depths_in = a.clone();
        
         // Bin masks
         cv::Mat valid_near = (depths_in > 0.1f) & (depths_in <= 15.0f);
         cv::Mat valid_med = (depths_in > 15.0f) & (depths_in <= 30.0f);
         cv::Mat valid_far = (depths_in > 30.0f);
        
         cv::Mat s1_depths = depths_in.clone();
          
         // Multi-scale dilation
         cv::Mat dilated_far, dilated_med, dilated_near;
         cv::Mat masked_far, masked_med, masked_near;
         cv::Mat valid_far_float, valid_med_float, valid_near_float;
         valid_far.convertTo(valid_far_float, s1_depths.type(), 1.0 / 255.0);
         valid_med.convertTo(valid_med_float, s1_depths.type(), 1.0 / 255.0);
         valid_near.convertTo(valid_near_float, s1_depths.type(), 1.0 / 255.0);
         cv::multiply(s1_depths, valid_far_float, masked_far);
         cv::multiply(s1_depths, valid_med_float, masked_med);
         cv::multiply(s1_depths, valid_near_float, masked_near);

         // Dilate all masks
         cv::dilate(masked_far, dilated_far, kernel_far);
         cv::dilate(masked_med, dilated_med, kernel_med);
         cv::dilate(masked_near, dilated_near, kernel_near);

         valid_near = (dilated_near > 0.1f);
         valid_med = (dilated_med > 0.1f);
         valid_far = (dilated_far > 0.1f);

         // Combine all the part of the image
         cv::Mat combined = s1_depths.clone();
         dilated_far.copyTo(combined, valid_far);
         dilated_med.copyTo(combined, valid_med);
         dilated_near.copyTo(combined, valid_near);
        
         // Morph close
         cv::Mat closed;
         cv::Mat kernel5 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
         morphologyEx(combined, closed, cv::MORPH_CLOSE, kernel5);

         // Median blur (preserve valid pixels)
         cv::Mat median_blur;
         cv::medianBlur(closed, median_blur, 5);
         closed.copyTo(median_blur, closed <= 0.1f);  // keep invalid pixels unblurred

         // Top mask
         cv::Mat top_mask = cv::Mat::ones(a.size(), CV_8U);
         for (int col = 0; col < median_blur.cols; ++col) {
            for (int row = 0; row < median_blur.rows; ++row) {
                if (median_blur.at<float>(row, col) > 0.1f) {
                    top_mask(cv::Range(0, row), cv::Range(col, col + 1)) = 0;
                    break;
                }
            }
         }

         // Fill holes (first pass)
         cv::Mat full9 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(9, 9));
         cv::Mat dilated;
         cv::dilate(median_blur, dilated, full9);
         cv::Mat filled = median_blur.clone();
         dilated.copyTo(filled, (filled <= 0.1f) & top_mask);
        
         // Iterative dilation to fill large holes
         cv::Mat iter_blurred = filled.clone();
         cv::Mat full5 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
         for (int i = 0; i < 6; ++i) {
            dilate(iter_blurred, dilated, full5);
            dilated.copyTo(iter_blurred, (iter_blurred <= 0.1f) & top_mask);
         }

         // Median blur
         cv::Mat blurred;
         cv::medianBlur(iter_blurred, blurred, 5);
         blurred.copyTo(iter_blurred, (iter_blurred > 0.1f) & top_mask);

         // Final blur
         bilateralFilter(iter_blurred, blurred, 5, 0.5, 2.0);
         blurred.copyTo(iter_blurred, (iter_blurred > 0.1f) & top_mask);
            
         cv::Mat output = iter_blurred.clone();

         // Inpaint
         // Threshold and conversion in binary
         cv::Mat binary;
         cv::threshold(output, binary, 0.1f, 255, cv::THRESH_BINARY);
         binary.convertTo(binary, CV_8U);

         // Find the outlines
         cv::Mat flood_filled = binary.clone();
         cv::floodFill(flood_filled, cv::Point(0, 0), 255);
         cv::floodFill(flood_filled, cv::Point(flood_filled.cols-1, 0), 255);
         cv::floodFill(flood_filled, cv::Point(0, flood_filled.rows-1), 255);
         cv::floodFill(flood_filled, cv::Point(flood_filled.cols-1, flood_filled.rows-1), 255);

         cv::Mat flood_inv;
         cv::bitwise_not(flood_filled, flood_inv);

         // Holes = flood_inv & original inverted (to stay inside the image)
         cv::Mat holes = flood_inv & (~binary);

         // Remove smalls artifacts
         cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
         cv::morphologyEx(holes, holes, cv::MORPH_OPEN, kernel);

         cv::Mat dilated_bis;
         cv::Mat edgeKernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
         cv::dilate(holes, dilated_bis, edgeKernel);

         cv::Mat inpaint_mask = dilated_bis.clone();
         cv::dilate(inpaint_mask, inpaint_mask, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)));

         cv::inpaint(output, inpaint_mask, inpainted, 3, cv::INPAINT_NS);

         cv::Mat border_mask;
         cv::erode(inpaint_mask, border_mask, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
         border_mask = inpaint_mask - border_mask;

         cv::Mat smooth;
         cv::medianBlur(inpainted, smooth, 3);
         smooth.copyTo(inpainted, border_mask);

         double min, max;
         cv::minMaxLoc(smooth, &min, &max);
         
         for (cv::MatIterator_<float> it = smooth.begin<float>(), it_end = smooth.end<float>(); it != it_end; ++it) {
            if (*it == 0.0f) *it = max; // X est ta valeur de remplacement
         }

         // Clamp
         cv::Mat clipped;
         cv::threshold(smooth, clipped, max, max, cv::THRESH_TRUNC);     // Max clamp
         cv::threshold(clipped, clipped, min, min, cv::THRESH_TOZERO);      // Min clamp


         if(DataQueue.IsEmpty() || MsgQueue2.IsEmpty()){
            // Full Data and optimisations sent to the Displacement Node
            int dataSize = clipped.total() * clipped.elemSize();
            TSharedPtr<TArray<uint8>> DataCopy = MakeShared<TArray<uint8>>();
            DataCopy->AddUninitialized(dataSize);
            FMemory::Memcpy(DataCopy->GetData(), clipped.data, dataSize);
            if(DataQueue.IsEmpty()){
               DataQueue.Enqueue(DataCopy);
               bPendingDepthUpdate = true;
            }

            // Data before remap sent to the RGB for transformation of color image in depth frame
            int dataSize2 = clipped.total() * clipped.elemSize();
            TSharedPtr<TArray<uint8>> DataCopy2 = MakeShared<TArray<uint8>>();
            DataCopy2->AddUninitialized(dataSize2);
            FMemory::Memcpy(DataCopy2->GetData(), clipped.data, dataSize2);
            if(MsgQueue2.IsEmpty()) {
               MsgQueue2.Enqueue(DataCopy2);
            }
         }
       }
    }
} // Fill the depth
cv::Mat WarpFromCameraToEye(const cv::Mat& rgb, const cv::Mat& depth,
                                float fx, float fy, float cx, float cy,
                                const cv::Matx33f& R, const cv::Vec3f& t)
{
   cv::Mat output(rgb.size(), rgb.type(), cv::Scalar::all(0));

   for (int v = 0; v < rgb.rows; ++v)
   {
      for (int u = 0; u < rgb.cols; ++u)
      {
         float Z = depth.at<uint16>(v, u) * 0.001f; // Kinect depth in mm → m
         if (Z <= 0.1f || Z > 5.0f) continue; // skip invalid depths

         // Back-project pixel to 3D (in camera space)
         float X = (u - cx) * Z / fx;
         float Y = (v - cy) * Z / fy;
         cv::Vec3f P_cam(X, Y, Z);

         // Transform to eye space
         cv::Vec3f P_eye = R * P_cam + t;

         // Project to 2D (same intrinsics, assuming eye uses same camera model)
         float u_eye = (P_eye[0] * fx / P_eye[2]) + cx;
         float v_eye = (P_eye[1] * fy / P_eye[2]) + cy;

         if (u_eye >= 0 && u_eye < rgb.cols && v_eye >= 0 && v_eye < rgb.rows)
         {
            // Bilinear interpolation optional; here we just round
            int u_e = static_cast<int>(u_eye);
            int v_e = static_cast<int>(v_eye);

            output.at<cv::Vec4b>(v_e, u_e) = rgb.at<cv::Vec4b>(v, u);
         }
      }
   }

   return output;
}
void SaveTextureToPNG(UTexture2D* Texture, const FString& Filename)
{
	if (!Texture) return;

	// Make sure the texture is readable by the CPU
	FTexture2DMipMap& Mip = Texture->GetPlatformData()->Mips[0];
	
	const int32 Width = Mip.SizeX;
	const int32 Height = Mip.SizeY;

	// Copy of the pixels
	TArray<FColor> Pixels;
	Pixels.AddUninitialized(Width * Height);
	FMemory::Memcpy(Pixels.GetData(), Mip.BulkData.LockReadOnly(), Width * Height * sizeof(FColor));

	Mip.BulkData.Unlock();

	// PNG Encoding
	IImageWrapperModule& ImageWrapperModule = FModuleManager::LoadModuleChecked<IImageWrapperModule>(FName("ImageWrapper"));
	TSharedPtr<IImageWrapper> ImageWrapper = ImageWrapperModule.CreateImageWrapper(EImageFormat::PNG);

	ImageWrapper->SetRaw(Pixels.GetData(), Pixels.Num() * sizeof(FColor), Width, Height, ERGBFormat::BGRA, 8);

	const TArray64<uint8>& PNGData = ImageWrapper->GetCompressed(100);

	// Save the file
	FString AbsolutePath = FPaths::ProjectDir() + Filename;
	FFileHelper::SaveArrayToFile(PNGData, *AbsolutePath);

	UE_LOG(LogTemp, Log, TEXT("Texture saved to: %s"), *AbsolutePath);
} // Allow you to save a Texture as an Image
void SaveIRTextureToGrayscale16bitTIFF(UTexture2D* Texture, const FString& Filename)
{
	if (!Texture) return;

	if (Texture->GetPixelFormat() != PF_G16)
	{
		UE_LOG(LogTemp, Error, TEXT("Unsupported pixel format. Expected PF_A16B16G16R16."));
		return;
	}

   // Make sure the texture is readable by the CPU
	FTexture2DMipMap& Mip = Texture->GetPlatformData()->Mips[0];
	const int32 Width = Mip.SizeX;
	const int32 Height = Mip.SizeY;
	const int32 PixelCount = Width * Height;

	const uint16* SrcData = static_cast<const uint16*>(Mip.BulkData.LockReadOnly());

	TArray<uint16> GrayscalePixels;
	GrayscalePixels.SetNumUninitialized(PixelCount);

	for (int32 i = 0; i < PixelCount; ++i)
	{
		uint16 A = SrcData[i]; // Alpha channel as IR
		GrayscalePixels[i] = A;
	}

	Mip.BulkData.Unlock();
	UE_LOG(LogTemp, Warning, TEXT("Trying to create texture with size: %d x %d x %d x %d"), Width, Height, PixelCount, GrayscalePixels.Num());
	// Convert Unreal path to ANSI for libtiff
	std::string StdFilename = TCHAR_TO_UTF8(*FPaths::ConvertRelativePathToFull(FPaths::ProjectDir() + Filename));
	TIFF* TiffFile = TIFFOpen(StdFilename.c_str(), "w");

	if (!TiffFile)
	{
		UE_LOG(LogTemp, Error, TEXT("Failed to open TIFF file for writing."));
		return;
	}

	TIFFSetField(TiffFile, TIFFTAG_IMAGEWIDTH, Width);
	TIFFSetField(TiffFile, TIFFTAG_IMAGELENGTH, Height);
	TIFFSetField(TiffFile, TIFFTAG_SAMPLESPERPIXEL, 1);
	TIFFSetField(TiffFile, TIFFTAG_BITSPERSAMPLE, 16);
	TIFFSetField(TiffFile, TIFFTAG_ORIENTATION, ORIENTATION_TOPLEFT);
	TIFFSetField(TiffFile, TIFFTAG_PLANARCONFIG, PLANARCONFIG_CONTIG);
	TIFFSetField(TiffFile, TIFFTAG_PHOTOMETRIC, PHOTOMETRIC_MINISBLACK);
	TIFFSetField(TiffFile, TIFFTAG_COMPRESSION, COMPRESSION_NONE); // No compression

	for (int32 Row = 0; Row < Height; ++Row)
	{
		void* Scanline = &GrayscalePixels[Row * Width];
		if (TIFFWriteScanline(TiffFile, Scanline, Row, 0) < 0)
		{
			UE_LOG(LogTemp, Error, TEXT("Failed to write TIFF scanline."));
			break;
		}
	}

	TIFFClose(TiffFile);

	UE_LOG(LogTemp, Log, TEXT("16-bit grayscale TIFF (IR) saved to: %s"), *Filename);
} // Allow you to save a Grayscale Texture 16 bits as an Image



/*----------------------------------------------------------------*/
//			 			     	      Unreal 	   		 	   			   //
/*----------------------------------------------------------------*/
AROS2SubscriberNode::AROS2SubscriberNode()
{
   
   // Allow the Tick Function (called every frame)
   PrimaryActorTick.bCanEverTick = true;
   PrimaryActorTick.bStartWithTickEnabled = true;

   // Creation of the ROS Node, used to gather datas from the Azure kinect
   Node = CreateDefaultSubobject<UROS2NodeComponent>(TEXT("ROS2NodeComponent"));
   Node->Name = TEXT("unreal_subscriber_node");
}

void AROS2SubscriberNode::BeginPlay()
{
   Super::BeginPlay();
   setenv("ROS_DOMAIN_ID", "20", 1);
   Node->Init();
   
   // Get the Mesh from the Dynamic Actor
   DynamicMesh = DynamicMeshActor->GetDynamicMeshComponent();
   
   if (DynamicMesh && BaseMaterial)
   {
      // Create the dynamic Material and assign it to the Mesh to dynamically change Texture
      DynamicMaterialInstance = UMaterialInstanceDynamic::Create(BaseMaterial, this);
      DynamicMesh->SetMaterial(0, DynamicMaterialInstance);
   }
   
   // Load the calibration file of the Azure Kinect
   // Located at ~/Documents/Unreal Project/<Project name>/calibration.txt
   FString AbsolutePath = FPaths::ProjectDir()+ "calibration.txt";
   std::ifstream rfile(*AbsolutePath, std::ios::binary);
   if (rfile.is_open())
   {
      std::vector<uint8> calibFile;
      // Read the content of the calibration file
      std::string content((std::istreambuf_iterator<char>(rfile)),std::istreambuf_iterator<char>());
      calibFile = std::vector<uint8>(content.begin(), content.end());
      rfile.close();
   
      // Initialise the calibration and transformation object from Azure SDK for RGB image transformation
      calib = k4a::calibration::get_from_raw(calibFile, K4A_DEPTH_MODE_WFOV_UNBINNED, K4A_COLOR_RESOLUTION_1536P);
      transformation = k4a::transformation(calib);
      
      // Create all the Subscriber used to get the RGB, Depth and IR Stream
      ROS2_CREATE_SUBSCRIBER(Node, this, RGBTopicName, UROS2ImgMsg::StaticClass(), &AROS2SubscriberNode::RGBCallback);
      ROS2_CREATE_SUBSCRIBER(Node, this, DepthTopicName, UROS2ImgMsg::StaticClass(), &AROS2SubscriberNode::DepthCallback);
      ROS2_CREATE_SUBSCRIBER(Node, this, IRTopicName, UROS2ImgMsg::StaticClass(), &AROS2SubscriberNode::IRCallback);
      ROS2_CREATE_SUBSCRIBER(Node, this, IMUTopicName, UROS2ImuMsg::StaticClass(), &AROS2SubscriberNode::IMUCallback);
      
      // Find dynamic variables of the Dynamic Mesh Blueprint
      TextureProp = FindFProperty<FObjectProperty>(DynamicMeshActor->GetClass(), "Displacement Map");
      IsDepthSimuEnabled = FindFProperty<FBoolProperty>(DynamicMeshActor->GetClass(), "DepthSimu");
      
      // Current Player
      PlayerPawn = UGameplayStatics::GetPlayerPawn(GetWorld(), 0);
       
      // Start the Thread used by the depth inpainting method to reduce lag
      InpaintThread = std::thread([this]() { InpaintDepth(); });
      //InpaintThread.detach();
   }
}
void AROS2SubscriberNode::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
   // Stop the thread
   bStopThread = true;
   QueueCV.notify_all();
   if (InpaintThread.joinable()) {
      InpaintThread.join();
   }
     
   Super::EndPlay(EndPlayReason);
}
void AROS2SubscriberNode::Tick(float DeltaSeconds)
{
   Super::Tick(DeltaSeconds);

    // If the RGB Texture need to be updated
    if (bPendingTextureUpdate && DynamicMaterialInstance) {
       Texture->UpdateResource();
       DynamicMaterialInstance->SetTextureParameterValue(TextureParameterName, Texture);
       bPendingTextureUpdate = false;
    }
    
    // If the Depth Texture need to be updated
    if (bPendingDepthUpdate && DynamicMaterialInstance) {
       // Empty the queue (used to communicate between the threads)
       TSharedPtr<TArray<uint8>> DataCopy;
       if (DataQueue.IsEmpty()) {
          return;
       }
       if (DataQueue.Dequeue(DataCopy) && DataCopy.IsValid()) {
          // Update texture
          void* DepthData = Depth->GetPlatformData()->Mips[0].BulkData.Lock(LOCK_READ_WRITE);
          FMemory::Memcpy(DepthData, DataCopy->GetData(), DataCopy->Num());
          Depth->GetPlatformData()->Mips[0].BulkData.Unlock();
    
          // Assign Texture to the Displacement Node of the Dynamic Mesh Blueprint
          if (TextureProp) {
             TextureProp->SetPropertyValue_InContainer(DynamicMeshActor, Depth);
          }
          
          Depth->UpdateResource();
          DynamicMaterialInstance->SetTextureParameterValue(TextureParameterName2, Depth);
          DataQueue.Empty();
          bPendingDepthUpdate = false;
       }
    }
    
    // If the IR Texture need to be updated
    if (bPendingIRUpdate && DynamicMaterialInstance) {
       IR->UpdateResource();
       DynamicMaterialInstance->SetTextureParameterValue(TextureParameterName3, IR);
       bPendingIRUpdate = false;
    }
    
    // Get the UI elements OR Update their value
    if (!Slider || !MultiView || !RGBView || !DepthView || !IRView || !DepthSimu || !TpCenter || !FixView)
    {
       if (PlayerPawn)
       {
          // Navigate through the Widget of the Pawn to find the UI
          TArray<UWidgetComponent*> WidgetComponents;
          PlayerPawn->GetComponents<UWidgetComponent>(WidgetComponents);
          for (UWidgetComponent* WidgetComp : WidgetComponents)
          {
             FString name = WidgetComp->GetName();
             if (WidgetComp->GetName().Contains(TEXT("Widget")))
             {
                UUserWidget* Widget = WidgetComp->GetUserWidgetObject();
                if (!Widget) continue;
    
                // Save all the useful Widgets
                Slider = Cast<USlider>(Widget->GetWidgetFromName(TEXT("Slider_0")));
                MultiView = Cast<UCheckBox>(Widget->GetWidgetFromName(TEXT("MultiViewCheckBox")));
                RGBView = Cast<UCheckBox>(Widget->GetWidgetFromName(TEXT("RGBViewCheckBox")));
                DepthView = Cast<UCheckBox>(Widget->GetWidgetFromName(TEXT("DepthViewCheckBox")));
                IRView = Cast<UCheckBox>(Widget->GetWidgetFromName(TEXT("IRViewCheckBox")));
                DepthSimu = Cast<UCheckBox>(Widget->GetWidgetFromName(TEXT("DepthSimuCheckBox")));
                TpCenter = Cast<UCheckBox>(Widget->GetWidgetFromName(TEXT("TP")));
                FixView = Cast<UCheckBox>(Widget->GetWidgetFromName(TEXT("Fix")));
                LatencySlider = Cast<USlider>(Widget->GetWidgetFromName(TEXT("Latency")));
                LatencyText = Cast<UTextBlock>(Widget->GetWidgetFromName(TEXT("LatencyText")));
                if (Slider && MultiView && RGBView && DepthView && IRView && DepthSimu && TpCenter && FixView && LatencySlider && LatencyText)
                {
                   UE_LOG(LogTemp, Warning, TEXT("All UI Elements Found"));
                   break;
                }
             }
          }
       }
    }
    else
    {
       // Check Slider Value
       if (Slider && DynamicMaterialInstance) {
          float Value = Slider->GetValue();
          // If Value as changed since last time
          if (LastBlendValue != Value) {
             // Update the Dynamic Material accordingly to the Slider value
             DynamicMaterialInstance->SetScalarParameterValue(BlendParameterName, Value);
             LastBlendValue = Value;
          }
       }
    
       // Check MultiView Checkbox
       if (MultiView && DynamicMaterialInstance) {
          bool Value = MultiView->IsChecked();
          // If Checked as changed since last time
          if (LastMultiView != Value) {
             // Update the Dynamic Material accordingly to the Checkbox state
             DynamicMaterialInstance->SetScalarParameterValue(MultiViewParameterName, Value);
             LastMultiView = Value;
          }
       }
    
       // Check RGB Checkbox
       if (RGBView && DynamicMaterialInstance) {
          bool Value = RGBView->IsChecked();
          // If Checked as changed since last time
          if (LastRGBView != Value) {
             // Update the Dynamic Material accordingly to the Checkbox state
             DynamicMaterialInstance->SetScalarParameterValue(RGBParameterName, Value);
             LastRGBView = Value;
          }
       }
    
       // Check Depth Checkbox
       if (DepthView && DynamicMaterialInstance) {
          bool Value = DepthView->IsChecked();
          // If Checked as changed since last time
          if (LastDepthView != Value) {
             // Update the Dynamic Material accordingly to the Checkbox state
             DynamicMaterialInstance->SetScalarParameterValue(DepthParameterName, Value);
             LastDepthView = Value;
          }
       }
    
       // Check IR Checkbox
       if (IRView && DynamicMaterialInstance) {
          bool Value = IRView->IsChecked();
          // If Checked as changed since last time
          if (LastIRView != Value) {
             // Update the Dynamic Material accordingly to the Checkbox state
             DynamicMaterialInstance->SetScalarParameterValue(IRParameterName, Value);
             LastIRView = Value;
          }
       }
    
       // Check Depth Simulation Checkbox
       if (DepthSimu && DynamicMaterialInstance) {
          bool Value = DepthSimu->IsChecked();
          // If Checked as changed since last time
          if (LastDepthSimu != Value) {
             DynamicMaterialInstance->SetScalarParameterValue(DepthSimuParameterName, Value);
             if (IsDepthSimuEnabled) {
                // Update the Dynamic Material accordingly to the Checkbox state
                IsDepthSimuEnabled->SetPropertyValue_InContainer(DynamicMeshActor, Value);
                if (IsDepthSimuEnabled->GetPropertyValue_InContainer(DynamicMeshActor) == 1) {
                   DynamicMeshActor->SetActorScale3D(FVector(0.1f, 0.1f, 0.1f));
                   DynamicMeshActor->SetActorLocation(FVector(-100, 0, 120));
                } else {
                   DynamicMeshActor->SetActorScale3D(FVector(0.6f, -0.6f, 0.6f));
                   DynamicMeshActor->SetActorLocation(FVector(-100, 0, 80));
                }
                if (TpCenter)
                   TpCenter->SetCheckedState(ECheckBoxState::Checked);
             }
             LastDepthSimu = Value;
          }
       }
    
       // Check Teleport to Center Button (Hidden Checkbox)
       if (TpCenter && DynamicMeshActor) {
          bool Value = TpCenter->IsChecked();
          if (Value && PlayerPawn) {
             // Get the position of the Dynamic Mesh in the Scene
             FVector MeshPosition = DynamicMeshActor->GetActorLocation();
    
             // Get the position of the VR Headset in the Scene
             TArray<UCameraComponent*> CameraComponents;
             PlayerPawn->GetComponents<UCameraComponent>(CameraComponents);
             
             // Compute the Mesh Position Minus the Camera Position to align the two elements
             for (UCameraComponent* CameraComp : CameraComponents) {
                if (IsDepthSimuEnabled->GetPropertyValue_InContainer(DynamicMeshActor) == 1) {
                   MeshPosition -= FVector(CameraComp->GetRelativeLocation().X,CameraComp->GetRelativeLocation().Y, CameraComp->GetRelativeLocation().Z);
                } else {
                   MeshPosition -= FVector(CameraComp->GetRelativeLocation().X+65,CameraComp->GetRelativeLocation().Y, CameraComp->GetRelativeLocation().Z-10);
                }
             }
    
             // Teleport the User to our custom center
             PlayerPawn->TeleportTo(MeshPosition,FRotator(0,0,0));
             TpCenter->SetCheckedState(ECheckBoxState::Unchecked);
          }
       }

       // Check Latency Slider Value
       if (LatencySlider && DynamicMaterialInstance) {
          float Value = LatencySlider->GetValue();
          // If Value as changed since last time
          if (LastLatency != Value) {
             Latency = FTimespan(Value * 10000000);
             LatencyTime = FDateTime(1, 1, 1);
             CamLocations.Empty();
             CamRotations.Empty();
             SpringLocations.Empty();
             LatencyText->SetText(FText::AsNumber(Value));
             LastLatency = Value;
          }
       }
       
       if (FixView && DynamicMeshActor) {
          bool bValue = FixView->IsChecked();
          if (bValue && PlayerPawn) {
             if (LatencyTime == FDateTime(1, 1, 1)) {
                LatencyTime = FDateTime::Now();
             }
             else {
                if (FDateTime::Now() - LatencyTime > Latency)
                {
                   FVector NewLocationCam, NewLocationSpring, NewLocation;
                   CamLocations.Dequeue(NewLocationCam);
                   SpringLocations.Dequeue(NewLocationSpring);
                   if (IsDepthSimuEnabled->GetPropertyValue_InContainer(DynamicMeshActor) == 1) {
                      NewLocation = NewLocationCam;
                   } else {
                      NewLocation = NewLocationSpring;
                   }

                   FRotator NewRotationTemp;
                   CamRotations.Dequeue(NewRotationTemp);
                   FRotator NewRotation = FRotator(-NewRotationTemp.Roll, 90+NewRotationTemp.Yaw, 270+NewRotationTemp.Pitch); 

                   DynamicMeshActor->SetActorLocationAndRotation(NewLocation, NewRotation);
                }
             }
             TArray<UCameraComponent*> CameraComponents;
             TArray<USpringArmComponent*> SpringArmComponents;
             PlayerPawn->GetComponents<UCameraComponent>(CameraComponents);
             PlayerPawn->GetComponents<USpringArmComponent>(SpringArmComponents);
                
             if (CameraComponents.Num() > 0 && SpringArmComponents.Num() > 0) {
                UCameraComponent* CameraComp = CameraComponents[0];
                USpringArmComponent* SpringArmComp = SpringArmComponents[0];
                CamLocations.Enqueue(CameraComp->GetComponentLocation() + FVector(10, 0, 0));
                SpringLocations.Enqueue(SpringArmComp->GetComponentLocation());
                CamRotations.Enqueue(CameraComp->GetComponentRotation());
             }
          } else {
             // Reset position
             if (IsDepthSimuEnabled->GetPropertyValue_InContainer(DynamicMeshActor) == 1) {
                DynamicMeshActor->SetActorScale3D(FVector(0.1f, 0.1f, 0.1f));
                DynamicMeshActor->SetActorLocation(FVector(-100, 0, 120));
             } else {
                DynamicMeshActor->SetActorScale3D(FVector(0.6f, -0.6f, 0.6f));
                DynamicMeshActor->SetActorLocation(FVector(-100, 0, 80));
             }
             DynamicMeshActor->SetActorRotation(FRotator(0, 90, 270));
          }
       }
    }
}

  
  
/*----------------------------------------------------------------*/
//			 					         ROS 							         //
/*----------------------------------------------------------------*/
void AROS2SubscriberNode::RGBCallback(const UROS2GenericMsg* InMsg) {
   // Check if the received message is valid
   if (!IsValid(this)) return;
   if (!InMsg)    {
      UE_LOG(LogTemp, Error, TEXT("Received null message!"));
      return;
   }

   // Store the Message
   const UROS2ImgMsg* ImgMsg = Cast<UROS2ImgMsg>(InMsg);
   if (!ImgMsg) return;
   FROSImg Msg;
   ImgMsg->GetMsg(Msg);

   // Check if the Color Texture is already initialised
   if ((!Texture || WidthTexture != Msg.Width || HeightTexture != Msg.Height) && WidthDepth != 0 && HeightDepth != 0) {
      // Image Size
      WidthTexture = Msg.Width;
      HeightTexture = Msg.Height;

      // Create the Color Texture and configure it
      UE_LOG(LogTemp, Warning, TEXT("Trying to create texture with size: %d x %d"), WidthDepth, HeightDepth);
      Texture = UTexture2D::CreateTransient(WidthDepth, HeightDepth, PF_B8G8R8A8);
      if (!Texture) {
         UE_LOG(LogTemp, Error, TEXT("Failed to create texture"));
         return;
      }
      Texture->SRGB = true;
      Texture->AdjustRGBCurve = 2.2f;
      BGRADataTexture.Reserve(WidthDepth * HeightDepth * 4);
      Texture->UpdateResource();
   }

   // Get the depth image sent by the Depth Filling Thread
   TSharedPtr<TArray<uint8>> DataCopy;
   if(Texture && MsgQueue2.Dequeue(DataCopy) && DataCopy.IsValid()){
      MsgQueue2.Empty();
      // Unwrap the Message information inside an Unreal Engine variable
      RawDataTexture = Msg.Data;
      BGRADataTexture.Empty();

      // Create custom OpenCV image for the Depth and convert it to 16 bits using the inverse function of the Azure Kinect ROS Publisher
      cv::Mat depth_frame_buffer_mat(HeightDepth, WidthDepth, CV_32FC1, DataCopy->GetData());
      if (IsDepthSimuEnabled->GetPropertyValue_InContainer(DynamicMeshActor) != 1) {
         depth_frame_buffer_mat.setTo(1);
      }
      cv::Mat new_image(HeightDepth, WidthDepth, CV_16UC1);
      depth_frame_buffer_mat.convertTo(new_image, CV_16UC1, 1000.0f);
      cv::Mat depth_uint16_contig = new_image.clone();

      // Create the K4A Images used for the frame transformation
      k4a::image b = k4a::image::create_from_buffer(
         K4A_IMAGE_FORMAT_COLOR_BGRA32,
         WidthTexture, HeightTexture,
         WidthTexture * 4,
         RawDataTexture.GetData(),
         HeightTexture * WidthTexture * 4,
         nullptr,nullptr);
      k4a::image b2 = k4a::image::create_from_buffer(
         K4A_IMAGE_FORMAT_DEPTH16,
         WidthDepth, HeightDepth,
         WidthDepth * sizeof(uint16),
         depth_uint16_contig.data,
         HeightDepth * WidthDepth * sizeof(uint16),
         nullptr, nullptr);

      // Transform the RGB image into the Depth Frame
      k4a::image img = transformation.color_image_to_depth_camera(b2, b);
      cv::Mat transformed(img.get_height_pixels(), img.get_width_pixels(), CV_8UC4, img.get_buffer());

      if (CamAbsoluteRotation != FRotator(0, 0, 0))
      {
         FQuat Quat = CamAbsoluteRotation.Quaternion();
         FMatrix UnrealRotMatrix = FRotationMatrix::Make(Quat);
         cv::Matx33f R;
         for (int row = 0; row < 3; ++row)
            for (int col = 0; col < 3; ++col)
               R(row, col) = UnrealRotMatrix.M[row][col];  // transpose if needed depending on row-major / col-major
         cv::Vec3f t(0.0f, -0.06f, -0.06f);
         cv::Mat output = WarpFromCameraToEye(transformed, depth_uint16_contig, 503.23f, 503.305f, 523.143f, 513.149f, R, t);
      
         BGRADataTexture.SetNumUninitialized(output.rows * output.cols * 4);
         FMemory::Memcpy(BGRADataTexture.GetData(), output.data, output.rows * output.cols * 4);
      }else
      {
         BGRADataTexture.SetNumUninitialized(transformed.rows * transformed.cols * 4);
         FMemory::Memcpy(BGRADataTexture.GetData(), transformed.data, transformed.rows * transformed.cols * 4);
      }

      // Stores the data inside the Color Texture
      void* TextureData = Texture->GetPlatformData()->Mips[0].BulkData.Lock(LOCK_READ_WRITE);
      FMemory::Memcpy(TextureData, BGRADataTexture.GetData(), BGRADataTexture.Num());
      Texture->GetPlatformData()->Mips[0].BulkData.Unlock();

      // Uncomment to save the Texture to a PNG File
      //SaveTextureToPNG(Texture, "Texture.png");

      // Call the Updater inside the Tick
      if (Texture) {
         bPendingTextureUpdate = true;
      }
   }
} // Called when an RGB/Color image is received
void AROS2SubscriberNode::DepthCallback(const UROS2GenericMsg* InMsg) {
   // Check if the received message is valid
   if (!IsValid(this)) return;
   if (!InMsg)    {
      UE_LOG(LogTemp, Error, TEXT("Received null message!"));
      return;
   }

   // Store the Message
   const UROS2ImgMsg* ImgMsg = Cast<UROS2ImgMsg>(InMsg);
   if (!ImgMsg) return;
   FROSImg Msg;
   ImgMsg->GetMsg(Msg);

   // Check if the Depth Texture is already initialised
   if (!Depth || WidthDepth != Msg.Width || HeightDepth != Msg.Height) {
      // Image Size
      WidthDepth = Msg.Width;
      HeightDepth = Msg.Height;

      // Create the Depth Texture and configure it
      UE_LOG(LogTemp, Warning, TEXT("Trying to create depth with size: %d x %d"), WidthDepth, HeightDepth);
      Depth = UTexture2D::CreateTransient(WidthDepth, HeightDepth, PF_R32_FLOAT);
      if (!Depth) {
         UE_LOG(LogTemp, Error, TEXT("Failed to create depth"));
         return;
      }
      Depth->SRGB = false;
      Depth->MipGenSettings = TMGS_NoMipmaps;
      Depth->CompressionSettings = TC_HDR;
      BGRADataDepth.Reserve(WidthDepth * HeightDepth * 4);
      Depth->UpdateResource();
   }

   // Unwrap the Message information inside an Unreal Engine variable
   BGRADataDepth = Msg.Data;

   // Add the Depth Image to the queue for further infill and use by Depth Infill Thread
   if (MsgQueue.IsEmpty())
   {
      std::lock_guard<std::mutex> lock(QueueMutex);
      MsgQueue.Enqueue(BGRADataDepth);
      QueueCV.notify_all();
   }
} // Called when a Depth image is received
void AROS2SubscriberNode::IRCallback(const UROS2GenericMsg* InMsg) {
   // Check if the received message is valid
   if (!IsValid(this)) return;
   if (!InMsg)    {
      UE_LOG(LogTemp, Error, TEXT("Received null message!"));
      return;
   }

   // Store the Message
   const UROS2ImgMsg* ImgMsg = Cast<UROS2ImgMsg>(InMsg);
   if (!ImgMsg) return;
   FROSImg Msg;
   ImgMsg->GetMsg(Msg);

   // Check if the IR Texture is already initialised
   if (!IR) {
      // Create the IR Texture and configure it
      UE_LOG(LogTemp, Warning, TEXT("Trying to create IR with size: %d x %d"), WidthDepth, HeightDepth);
      IR = UTexture2D::CreateTransient(WidthDepth, HeightDepth, PF_G16);
      if (!IR) {
         UE_LOG(LogTemp, Error, TEXT("Failed to create IR"));
         return;
      }
      IR->SRGB = false;
      BGRADataIR.SetNumUninitialized(WidthDepth * HeightDepth * 2);
      IR->UpdateResource();
   }

   // Unwrap the Message information inside an Unreal Engine variable
   RawDataIR = Msg.Data;

   // Convert the message received as an uint8 to uint16
   int j = 0;
   for (int i = 0; i < RawDataIR.Num(); i += 2)
   {
      BGRADataIR[j] = (RawDataIR[i] | RawDataIR[i + 1] << 8);
      j+=1;
   }

   // Stores the data inside the IR Texture
   void* IRData = IR->GetPlatformData()->Mips[0].BulkData.Lock(LOCK_READ_WRITE);
   FMemory::Memcpy(IRData, BGRADataIR.GetData(), BGRADataIR.Num());
   IR->GetPlatformData()->Mips[0].BulkData.Unlock();

   // Uncomment to save the IR Texture to a 16 bits TIFF File
   //SaveIRTextureToGrayscale16bitTIFF(IR, "IR.png");

   // Call the Updater inside the Tick
   if (IR) {
      bPendingIRUpdate = true;
   }
} // Called when an IR image is received
void AROS2SubscriberNode::IMUCallback(const UROS2GenericMsg* InMsg) {
   // Check if the received message is valid
   if (!IsValid(this)) return;
   if (!InMsg)    {
      UE_LOG(LogTemp, Error, TEXT("Received null message!"));
      return;
   }

   // Store the Message
   const UROS2ImuMsg* ImuMsg = Cast<UROS2ImuMsg>(InMsg);
   if (!ImuMsg) return;
   FROSImu Msg;
   ImuMsg->GetMsg(Msg);

   FQuat Orientation;
   float Alpha = 0.98f;
   
   FVector Accel = Msg.LinearAcceleration;
   FVector Gyro = Msg.AngularVelocity;
   float DeltaTime = 0.1f;

   float Angle = Gyro.Size() * DeltaTime;
   FVector Axis = Gyro.GetSafeNormal();

   FQuat DeltaRot = FQuat(Axis, Angle);
   Orientation = Orientation * DeltaRot;
   Orientation.Normalize();

   FVector GravityDir = Accel.GetSafeNormal();
   FVector Up = -GravityDir;
   
   FVector Forward = FVector::CrossProduct(FVector::RightVector, Up).GetSafeNormal();
   FMatrix AccRotationMatrix = FRotationMatrix::MakeFromXZ(Forward, Up);
   FQuat AccQuat = FQuat(AccRotationMatrix);

   Orientation = FQuat::Slerp(AccQuat, Orientation, Alpha);
   Orientation.Normalize();
   //CamAbsoluteRotation = Orientation.Rotator() * 100;

   TArray<UCameraComponent*> CameraComponents;
   PlayerPawn->GetComponents<UCameraComponent>(CameraComponents);
            
   // Compute the Mesh Position Minus the Camera Position to align the two elements
   for (UCameraComponent* CameraComp : CameraComponents) {
      //CamAbsoluteRotation = FRotator(-CamAbsoluteRotation.Roll, CamAbsoluteRotation.Yaw, CamAbsoluteRotation.Pitch);
   }
} // Called when an IMU information is received