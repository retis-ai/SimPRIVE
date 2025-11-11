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


#include "Rover.h"

// Sets default values
ARover::ARover()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = false;

	//RootComponent = CreateDefaultSubobject<USceneComponent>(TEXT("RootComponent"));
	RootComponent->SetMobility(EComponentMobility::Movable);

	// Create and attach the static mesh component
	EditableStaticMeshComponent = GetStaticMeshComponent();// CreateDefaultSubobject<UStaticMeshComponent>(TEXT("EditableStaticMeshComponent"));
	if (EditableStaticMeshComponent)
	{
		EditableStaticMeshComponent->SetMobility(EComponentMobility::Movable);
		EditableStaticMeshComponent->SetSimulatePhysics(false);
		EditableStaticMeshComponent->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
		static ConstructorHelpers::FObjectFinder<UStaticMesh> MeshAsset(TEXT("/SimPRIVE/Meshes/rover.rover"));
		if (MeshAsset.Succeeded())
		{
			EditableStaticMeshComponent->SetStaticMesh(MeshAsset.Object);
			UE_LOG(LogTemp, Warning, TEXT("Static Mesh found correctly"));
		}
		else
		{
			UE_LOG(LogTemp, Warning, TEXT("Static Mesh NOT FOUND"));
		}
	}
	else
	{
		UE_LOG(LogTemp, Error, TEXT("EditableStaticMeshComponent is null"));
	}

	FVector LiDARStartLocation = FVector(0.0, 0.0f, 30.0f);
	FQuat LiDARStartOrientation = FQuat(FRotator(0.0f, 0.0f, 0.0f));
	FVector LiDARStartScale = FVector(.2f, .2f, .20f);
	LiDARTransform.SetLocation(LiDARStartLocation);
	LiDARTransform.SetRotation(LiDARStartOrientation);
	LiDARTransform.SetScale3D(LiDARStartScale);


	CameraTransform.SetLocation(FVector(20.0, 0.0f, 30.0f));
	CameraTransform.SetRotation(FQuat(FRotator(0.0f, 0.0f, 0.0f))); // Example rotation
	CameraTransform.SetScale3D(FVector(.2f, .2f, .20f)); // Example scale


}

// Called when the game starts or when spawned
void ARover::BeginPlay()
{
	Super::BeginPlay();
	Init();
}

// Called every frame
void ARover::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

}



void ARover::Init()
{
	UWorld* World = GetWorld();

	// Attach static mesh component to root
	SetRootComponent(EditableStaticMeshComponent);
	EditableStaticMeshComponent->OnComponentBeginOverlap.AddDynamic(this, &ARover::OnBeginOverlap);

	this->RegisterAllComponents();

	InitSensors();
}

void ARover::InitSensors()
{
	UWorld* World = GetWorld();

	Lidar = World->SpawnActor<ALidarActor>(ALidarActor::StaticClass(), EditableStaticMeshComponent->GetComponentLocation(), EditableStaticMeshComponent->GetComponentRotation());
	Lidar->AttachToComponent(EditableStaticMeshComponent, FAttachmentTransformRules::KeepRelativeTransform);
	Lidar->SetActorRelativeTransform(LiDARTransform);

	Lidar->SetVerticalFOV(30.0f);
	Lidar->SetHorizontalFOV(360.0f);
	Lidar->SetAngularResolution(1.0f);
	Lidar->SetRange(25.0f * 100);
	Lidar->RecomputeTrigVars();


	Camera = World->SpawnActor<ACameraSensor>(ACameraSensor::StaticClass(), EditableStaticMeshComponent->GetComponentLocation(), EditableStaticMeshComponent->GetComponentRotation());
	Camera->AttachToComponent(EditableStaticMeshComponent, FAttachmentTransformRules::KeepRelativeTransform);
	Camera->SetActorRelativeTransform(CameraTransform);

}



void ARover::CaptureImage(TArray<FColor>& BitMap)
{
	this->Camera->Capture(BitMap);
}


void ARover::CapturePointCloud(TArray<FVector>& PointCloudData)
{
	this->Lidar->CapturePointCloud(PointCloudData);
}



void ARover::OnBeginOverlap(
	UPrimitiveComponent* OverlappedComp,
	AActor* OtherActor,
	UPrimitiveComponent* OtherComp,
	int32 OtherBodyIndex,
	bool bFromSweep,
	const FHitResult& SweepResult
)
{
	// Handle the begin overlap event
	UE_LOG(LogTemp, Log, TEXT("Begin overlap detected!"));
	bCollision = true;
}


bool ARover::TryMove(FVector& Location, const FRotator& Rotation)
{
	Location.Z = Height;
	FRotator Rot = Rotation;
	bool bSuccess = SetActorLocationAndRotation(Location, Rotation, false, nullptr, ETeleportType::TeleportPhysics);
	FTransform WorldCamTransform;
	WorldCamTransform.SetLocation(Location);
	WorldCamTransform.SetRotation(FQuat(Rot));
	WorldCamTransform.SetScale3D(FVector(1.f, 1.f, 1.f));
    FTransform NewT = CameraTransform * WorldCamTransform;
	//Camera->SetActorLocationAndRotation(Location + CameraTransform.GetLocation(), WorldCamTransform.GetRotation());
	//Camera->SetActorLocationAndRotation(NewT.GetLocation(), NewT.GetRotation());
	return true;
}

