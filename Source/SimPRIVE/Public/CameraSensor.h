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

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"

#include "Components/SceneCaptureComponent2D.h"
#include "Engine/TextureRenderTarget2D.h"
#include "Components/DrawFrustumComponent.h"


#include "CameraSensor.generated.h"

USTRUCT()
struct FRenderRequestStruct {
	GENERATED_BODY()

	TArray<FColor> Image;
	FRenderCommandFence RenderFence;

	float TimeStamp;
	FVector Position;
	FRotator Orientation;

	FRenderRequestStruct() {

	}
};

UCLASS()
class SIMPRIVE_API ACameraSensor : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	ACameraSensor();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

	


	TQueue<FRenderRequestStruct*> RenderRequestQueue;
	UStaticMeshComponent* StaticMeshComponent;

	void InitCamera();

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

	bool SaveCapture(TArray<FColor>& BitMap);
	bool CaptureNonBlocking();
	void Capture(TArray<FColor>& BitMap);

	int32 GetImageWidth() const { return RenderTarget->GetSurfaceWidth(); }
	int32 GetImageHeight() const { return RenderTarget->GetSurfaceHeight(); }

	UPROPERTY(VisibleAnywhere, BlueprintReadWrite)
	USceneCaptureComponent2D* SceneCaptureComponent;
	UPROPERTY(VisibleAnywhere, BlueprintReadWrite)
	UTextureRenderTarget2D* RenderTarget;

	UDrawFrustumComponent* DrawFrustum;

	bool SaveCaptureToDisk(const TArray<FColor>& Bitmap, int32 Width, int32 Height, const FString& FilePath);

};
