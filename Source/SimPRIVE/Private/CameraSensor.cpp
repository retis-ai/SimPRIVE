// Copyright[2025][Scuola Superiore Sant'Anna]
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http ://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#include "CameraSensor.h"

#include "ImageUtils.h"
#include "IImageWrapper.h"
#include "IImageWrapperModule.h"
#include "Modules/ModuleManager.h"
#include "Misc/FileHelper.h"
#include "HAL/PlatformFilemanager.h"

// Sets default values
ACameraSensor::ACameraSensor()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = false;

	SceneCaptureComponent = CreateDefaultSubobject<USceneCaptureComponent2D>(TEXT("SceneCaptureComponent"));
	//StaticMeshComponent = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("StaticMeshComponent"));
	RootComponent = SceneCaptureComponent;
	//SceneCaptureComponent->AttachToComponent(RootComponent, FAttachmentTransformRules::KeepRelativeTransform);

	// Apply relative transformation
	//SceneCaptureComponent->SetRelativeLocation(FVector(40.0f, 0.0f, 20.0f)); // Example location
	//SceneCaptureComponent->SetRelativeRotation(FRotator(0.0f, 0.0f, 0.0f)); // Example rotation
	//SceneCaptureComponent->SetRelativeScale3D(FVector(1.0f, 1.0f, 1.0f)); // Example scale

	RenderTarget = CreateDefaultSubobject<UTextureRenderTarget2D>(TEXT("RenderTarget"));
	RenderTarget->InitAutoFormat(640, 480);
	SceneCaptureComponent->TextureTarget = RenderTarget;

	DrawFrustum = CreateDefaultSubobject<UDrawFrustumComponent>(TEXT("DrawFrustum"));
	DrawFrustum->SetupAttachment(SceneCaptureComponent);
	DrawFrustum->FrustumAngle = SceneCaptureComponent->FOVAngle;
	DrawFrustum->FrustumStartDist = 10.0f;
	DrawFrustum->FrustumEndDist = 1000.0f;

}

// Called when the game starts or when spawned
void ACameraSensor::BeginPlay()
{
	Super::BeginPlay();
	InitCamera();
	
}

void ACameraSensor::InitCamera()
{
	// Camera settings
	SceneCaptureComponent->bCaptureEveryFrame = true;
	SceneCaptureComponent->bAlwaysPersistRenderingState = true;

	SceneCaptureComponent->CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR;

	SceneCaptureComponent->PostProcessSettings.bOverride_AutoExposureMinBrightness = true;
	SceneCaptureComponent->PostProcessSettings.AutoExposureMinBrightness = 1.0f;
	SceneCaptureComponent->PostProcessSettings.bOverride_AutoExposureMaxBrightness = true;
	SceneCaptureComponent->PostProcessSettings.AutoExposureMaxBrightness = 1.0f;
	SceneCaptureComponent->PostProcessSettings.bOverride_AutoExposureBias = true;
	SceneCaptureComponent->PostProcessSettings.AutoExposureBias = 1.0f;

	SceneCaptureComponent->PostProcessSettings.bOverride_IndirectLightingIntensity = true;
	SceneCaptureComponent->PostProcessSettings.IndirectLightingIntensity = 3.0f;

	SceneCaptureComponent->PostProcessSettings.bOverride_BloomIntensity = true;
	SceneCaptureComponent->PostProcessSettings.BloomIntensity = 0.2f;
	SceneCaptureComponent->PostProcessSettings.bOverride_BloomThreshold = true;
	SceneCaptureComponent->PostProcessSettings.BloomThreshold = -1.0f;
	SceneCaptureComponent->PostProcessSettings.bOverride_BloomSizeScale = true;
	SceneCaptureComponent->PostProcessSettings.BloomSizeScale = 1.0f;
}

// Called every frame
void ACameraSensor::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

}

bool ACameraSensor::CaptureNonBlocking()
{
	FRenderRequestStruct* RenderReq = new FRenderRequestStruct();
	
	if (!IsValid(SceneCaptureComponent)) {
		return false;
	}

	FTextureRenderTargetResource* RenderTargetResource = RenderTarget->GameThread_GetRenderTargetResource();

	struct FReadSurfaceContext {
		FRenderTarget* SrcRenderTarget;
		TArray<FColor>* OutData;
		FIntRect Rect;
		FReadSurfaceDataFlags Flags;
	};

	FReadSurfaceContext ReadSurfaceContext = {
		RenderTargetResource,
		&(RenderReq->Image),
		FIntRect(0,0,RenderTargetResource->GetSizeXY().X, RenderTargetResource->GetSizeXY().Y),
		FReadSurfaceDataFlags(RCM_UNorm, CubeFace_MAX)
	};

	ENQUEUE_RENDER_COMMAND(SceneDrawCompletion)(
		[ReadSurfaceContext](FRHICommandListImmediate& RHICmdList) {
			RHICmdList.ReadSurfaceData(
				ReadSurfaceContext.SrcRenderTarget->GetRenderTargetTexture(),
				ReadSurfaceContext.Rect,
				*ReadSurfaceContext.OutData,
				ReadSurfaceContext.Flags
			);
		});

	RenderRequestQueue.Enqueue(RenderReq);

	RenderReq->RenderFence.BeginFence();


	return true;
}


// Called every frame
bool ACameraSensor::SaveCapture(TArray<FColor>& BitMap)
{
	if (!RenderRequestQueue.IsEmpty()) {
		// Peek the next RenderRequest from queue
		FRenderRequestStruct* NextRenderRequest = nullptr;
		RenderRequestQueue.Peek(NextRenderRequest);

		if (NextRenderRequest) //nullptr check
		{
			if (NextRenderRequest->RenderFence.IsFenceComplete()) // Check if rendering is done, indicated by RenderFence
			{
				// Immagine
				BitMap = NextRenderRequest->Image;

				// Delete the first element from RenderQueue
				RenderRequestQueue.Pop();
				delete NextRenderRequest;

				return true;

			}
		}
	}
	return false;

}

void ACameraSensor::Capture(TArray<FColor>& BitMap)
{

	if (!RenderTarget)
	{
		UE_LOG(LogTemp, Warning, TEXT("RenderTarget is null"));
		return;
	}

	


	// Get render target resource
	FTextureRenderTargetResource* RenderTargetResource = RenderTarget->GameThread_GetRenderTargetResource();
	if (!RenderTargetResource)
	{
		UE_LOG(LogTemp, Error, TEXT("RenderTargetResource is null"));
	}

	// Read pixels from render target
	int32 Width = RenderTarget->GetSurfaceWidth();
	int32 Height = RenderTarget->GetSurfaceHeight();
	BitMap.SetNum(Width * Height);
	//RenderTargetResource->ReadPixels(BitMap);

	CaptureNonBlocking();
	int WhileCount = 0;
	while (!SaveCapture(BitMap) && RenderTarget && RenderTargetResource && WhileCount < 10)
	{
		WhileCount++;
		//UE_LOG(LogTemp, Error, TEXT("WHILE"));

	}

	//DrawDebugCamera(
	//	GetWorld(),
	//	SceneCaptureComponent->GetComponentLocation(),
	//	SceneCaptureComponent->GetComponentRotation(),
	//	SceneCaptureComponent->FOVAngle,
	//	1000.0f, // Length of the frustum
	//	FColor::Green,
	//	false,
	//	0.1f
	//);


	//FString SavePath = FPaths::ProjectSavedDir() / TEXT("CameraCapture.png");
	//bool bSaved = SaveCaptureToDisk(BitMap, RenderTarget->SizeX, RenderTarget->SizeY, SavePath);

	//if (bSaved)
	//{
		//UE_LOG(LogTemp, Warning, TEXT("Image successfully saved to: %s"), *SavePath);
	//}
	//else
	//{
		//UE_LOG(LogTemp, Error, TEXT("Failed to save image to: %s"), *SavePath);
	//}


}


bool ACameraSensor::SaveCaptureToDisk(const TArray<FColor>& Bitmap, int32 Width, int32 Height, const FString& FilePath)
{
	IImageWrapperModule& ImageWrapperModule = FModuleManager::LoadModuleChecked<IImageWrapperModule>(FName("ImageWrapper"));
	TSharedPtr<IImageWrapper> ImageWrapper = ImageWrapperModule.CreateImageWrapper(EImageFormat::PNG);

	// Convert FColor array to raw BGRA data
	ImageWrapper->SetRaw(Bitmap.GetData(), Bitmap.GetAllocatedSize(), Width, Height, ERGBFormat::BGRA, 8);

	const TArray64<uint8>& PNGData = ImageWrapper->GetCompressed();

	return FFileHelper::SaveArrayToFile(PNGData, *FilePath);
}


