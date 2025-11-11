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

#include "Components/SplineComponent.h"
#include "PedestrianWrapper.h"
#include "ObjectWrapper.h"
#include "QRCodeWrapper.h"


#include "SpawningManager.generated.h"

UCLASS()
class SIMPRIVE_API ASpawningManager : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	ASpawningManager();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;




	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Paths")
	TArray<USplineComponent*> Paths;

	UPROPERTY(EditAnywhere, Category = "Library")
	TArray<USkeletalMesh*> SkeletalMeshLibrary;

	UPROPERTY(EditAnywhere, Category = "Library")
	TArray<UStaticMesh*> StaticMeshLibrary;

	UPROPERTY(EditAnywhere, Category = "Library")
	TArray<UAnimSequence*> AnimationLibrary;

	//UPROPERTY(EditAnywhere, Category = "Library")
	//UStaticMesh* QRCodeMesh;

	//UPROPERTY(EditAnywhere, Category = "Library")
	//UMaterialInterface* QRCodeDecalMaterial;

	UFUNCTION(CallInEditor, Category = "Custom")
	void AddNewSpline();

	// spawn
	void SpawnPedestrian();
	void SpawnObject();
	void SpawnQRCodeObject();

	// get spawn locations
	void GetRandomSpawnPoint(FVector& Location, FRotator& Rotation);
	void GetFreeSpawnPoint(FVector& Location, FRotator& Rotation);
	void GetRoverSpawnPoint(FVector& Location, FRotator& Rotation);
	void GetQRCodeSpawnPoint(FVector& Location, FRotator& Rotation);
	void ResetSpawnProbabilities();

	// delete
	void DeleteSpawnedObjects();


	// List of actors. All are spawned, but only a few are activated.
	TArray<APedestrianWrapper*> Pedestrians;
	TArray<AObjectWrapper*> Objects;
	TArray<AQRCodeWrapper*> QRCodes;


	TArray<float> SplineSpawnProbabilities;
	TArray<TArray<float>> DistanceSpawnProbabilities;

};
