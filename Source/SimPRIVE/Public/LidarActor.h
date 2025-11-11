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
#include "LidarActor.generated.h"

UCLASS()
class SIMPRIVE_API ALidarActor : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	ALidarActor();

	void CapturePointCloud(TArray<FVector>& PointCloudData);

	//void CaptureLaserScan(TArray<float>& Ranges);


	void SetVerticalFOV(float VFOV) { Settings.VerticalFOV = VFOV; };
	void SetHorizontalFOV(float HFOV) { Settings.FOV = HFOV; };
	void SetAngularResolution(float Res) { Settings.AngularResolution = Res; };
	void SetRange(float Range) { Settings.Range = Range; };
	void RecomputeTrigVars();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;


	void InitLidar();


	struct LidarSettings
	{
		float Range = 100.0f;
		float AngularResolution = 1.0f;
		float FOV = 360.0f;

		float VerticalFOV = 30.0f;

	};

	LidarSettings Settings;


	// Static mesh component
	UPROPERTY(VisibleAnywhere, Category = "Components")
	UStaticMeshComponent* StaticMeshComponent;

	TArray<float> CosAngles, SinAngles, CosVertAngles, SinVertAngles;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

};
