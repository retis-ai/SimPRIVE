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

#include "ROSIntegration/Classes/RI/Topic.h"
#include "ROSIntegration/Classes/ROSIntegrationGameInstance.h"
#include "ROSIntegration/Public/sensor_msgs/PointCloud2.h"
#include "ROSIntegration/Public/sensor_msgs/LaserScan.h"
#include "ROSIntegration/Public/sensor_msgs/Image.h"
#include "ROSIntegration/Public/nav_msgs/Odometry.h"
#include "ROSIntegration/Public/std_msgs/Bool.h"
#include "ROSIntegration/Public/std_msgs/Float32.h"

#include "Rover.h"
#include "SpawningManager.h"


#include "ROSOrchestrator.generated.h"

UCLASS()
class SIMPRIVE_API AROSOrchestrator : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	AROSOrchestrator();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;


	// Internal
	bool bProcessOdomMessages = false;
	bool JustResumed = false;

	FTransform LastRealPose;
	FTransform CurrentOffset;

	void InitEnvironment();

	void SubscribeToTopics();
	void AdvertiseTopics();

	void PublishImage(const TArray<FColor>& BitMap);
	void PublishPointCloud(const TArray<FVector>& PointCloudData);
	void PublishLaserScan(const TArray<float>& Ranges);
	void PublishCollision();
	//void PublishAdditionalInfo();

	void PauseGame();
	void ResumeGame();


	bool CheckSetup() const;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;


	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rover")
	ARover* Rover;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Spawning Manager")
	ASpawningManager* SpawnManager;

	// Subscribers and callbacks
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Topic Names")
	FString OdometryTopicName;

	UPROPERTY()
	UTopic* OdometryTopic;
	void OdometryCallback(TSharedPtr<FROSBaseMsg> Msg);

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Topic Names")
	FString ResetTopicName;

	UPROPERTY()
	UTopic* ResetTopic;
	void ResetCallback(TSharedPtr<FROSBaseMsg> Msg);

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Topic Names")
	FString PauseSimTopicName;

	UPROPERTY()
	UTopic* PauseSimTopic;
	void PauseSimCallback(TSharedPtr<FROSBaseMsg> Msg);

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Topic Names")
	FString ResumeSimTopicName;

	UPROPERTY()
	UTopic* ResumeSimTopic;
	void ResumeSimCallback(TSharedPtr<FROSBaseMsg> Msg);



	// Publishers
	UPROPERTY()
	UTopic* ImagePublisher;
	//TSharedPtr<UTopic> ImagePublisher;
	UPROPERTY()
	UTopic* LaserScanPublisher;

	UPROPERTY()
	UTopic* PointCloudPublisher;

	UPROPERTY()
	UTopic* CollisionPublisher;
	//TSharedPtr<UTopic> CollisionPublisher;
	UPROPERTY()
	UTopic* AdditionalInfoPublisher;


	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Spawning Manager")
	int NumSpawnObjects = 10;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Spawning Manager")
	int NumSpawnPedestrians = 0;

};
