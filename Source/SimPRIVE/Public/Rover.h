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
#include "Engine/StaticMeshActor.h"

#include "Components/StaticMeshComponent.h"

#include "CameraSensor.h"
#include "LidarActor.h"


#include "Rover.generated.h"

UCLASS()
class SIMPRIVE_API ARover : public AStaticMeshActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	ARover();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

	// Internal
	void Init();
	void InitSensors();

	// Collision flag
	bool bCollision = false;


public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

	// Static Mesh
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Components")
	UStaticMeshComponent* EditableStaticMeshComponent;

	// Sensors position
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensors")
	FTransform CameraTransform;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensors")
	FTransform LiDARTransform;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Level Height")
	float Height = 1.0f;

	// Sensors 
	ALidarActor* Lidar;
	ACameraSensor* Camera;
	void CaptureImage(TArray<FColor>& BitMap);
	void CapturePointCloud(TArray<FVector>& PointCloudData);


	// Collisions
	bool GetCollision() const { return bCollision; }

	void ResetCollision() { bCollision = false; }

	UFUNCTION()
	virtual void OnBeginOverlap(UPrimitiveComponent* OverlappedComp, AActor* OtherActor, UPrimitiveComponent* OtherComp, int32 OtherBodyIndex, bool bFromSweep, const FHitResult& SweepResult);


	// Movements
	bool TryMove(FVector& Location, const FRotator& Rotation);

};
