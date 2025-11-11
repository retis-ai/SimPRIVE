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


#include "LidarActor.h"

// Sets default values
ALidarActor::ALidarActor()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = false;

	// Create the static mesh component
	StaticMeshComponent = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("StaticMeshComponent"));

	// Attach the static mesh component to the root component
	RootComponent = StaticMeshComponent;

	static ConstructorHelpers::FObjectFinder<UStaticMesh> MeshAsset(TEXT("/Game/StarterContent/Shapes/Shape_Cylinder.Shape_Cylinder"));  ///Script/Engine.StaticMesh'/Game/StarterContent/Props/rover.rover'
	if (MeshAsset.Succeeded())
	{
		StaticMeshComponent->SetStaticMesh(MeshAsset.Object);
		UE_LOG(LogTemp, Warning, TEXT("Static Mesh found correctly"));
	}
	else
	{
		UE_LOG(LogTemp, Warning, TEXT("Static Mesh NOT FOUND"));

	}

	StaticMeshComponent->SetSimulatePhysics(false);
	StaticMeshComponent->SetCollisionEnabled(ECollisionEnabled::NoCollision);

}

// Called when the game starts or when spawned
void ALidarActor::BeginPlay()
{
	Super::BeginPlay();
	InitLidar();

}

// Called every frame
void ALidarActor::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

}



void ALidarActor::RecomputeTrigVars()
{
	CosAngles.Empty();
	SinAngles.Empty();
	CosVertAngles.Empty();
	SinVertAngles.Empty();

	for (float Angle = -Settings.FOV / 2; Angle < Settings.FOV / 2; Angle += Settings.AngularResolution)
	{
		CosAngles.Add(FMath::Cos(FMath::DegreesToRadians(Angle)));
		SinAngles.Add(FMath::Sin(FMath::DegreesToRadians(Angle)));
	}
	for (float vertAngle = -Settings.VerticalFOV / 2; vertAngle < Settings.VerticalFOV / 2; vertAngle += Settings.AngularResolution)
	{
		CosVertAngles.Add(FMath::Cos(FMath::DegreesToRadians(vertAngle)));
		SinVertAngles.Add(FMath::Sin(FMath::DegreesToRadians(vertAngle)));
	}

}

// TODO take from UE5 editor variables.
void ALidarActor::InitLidar()
{
	Settings.AngularResolution = 1.0f;
	Settings.FOV = 360.0f;
	Settings.Range = 25.0f * 100;
	Settings.VerticalFOV = 30.0f;


}



void ALidarActor::CapturePointCloud(TArray<FVector>& PointCloudData)
{

	PointCloudData.Empty();

	PointCloudData.SetNum(CosAngles.Num() * CosVertAngles.Num());


	UWorld* World = GetWorld();
	FCollisionQueryParams CollisionParams;
	//CollisionParams.AddIgnoredActor(World->GetFirstPlayerController()->GetPawn());
	FTransform LiDARTransform = GetActorTransform();
	FVector WorldStart = LiDARTransform.GetLocation(); // MountingPointTransform.TransformPosition(StartPoint);
	//WorldStart.Z += 30.0f;
	FTransform InverseTransform = LiDARTransform.Inverse();
	//UE_LOG(LogTemp, Warning, TEXT("World Start %s"), *WorldStart.ToString());
	ParallelFor(CosAngles.Num(), [&](int32 i)
	//for (int i = 0; i < CosAngles.Num(); i++)
		{
			for (int32 j = 0; j < CosVertAngles.Num(); ++j)
			{
				
				FVector RelativeRayDirection = FVector(CosVertAngles[j] * CosAngles[i], CosVertAngles[j] * SinAngles[i], SinVertAngles[j]);
				FVector WorldRayDirection = LiDARTransform.GetRotation().RotateVector(RelativeRayDirection);
				FVector WorldEnd = WorldStart + WorldRayDirection * Settings.Range; // MountingPointTransform.TransformPosition(EndPoint);
				//UE_LOG(LogTemp, Warning, TEXT("WorldRayDirection: %s"), *WorldRayDirection.ToString());


				FHitResult HitResult;
				int32 Index = i * CosVertAngles.Num() + j;
				if (World->LineTraceSingleByChannel(HitResult, WorldStart, WorldEnd, ECC_Visibility, CollisionParams))
				{
					FVector PCPoint = HitResult.ImpactPoint;

					FVector Translated = PCPoint - LiDARTransform.GetLocation();
					FVector LocalPoint = LiDARTransform.GetRotation().UnrotateVector(Translated);

					//FVector LocalPoint = InverseTransform.TransformPositionNoScale(PCPoint);

					PointCloudData[Index] = LocalPoint;

				}
				else
				{
					PointCloudData[Index] = FVector::Zero();
				}
			}
		});

}

/*
void ALidarActor::CaptureLaserScan(TArray<float>& Ranges)
{
	Ranges.Empty();
	UWorld* World = GetWorld();
	FVector StartPoint(0.0, 0.0, 40.0);
	FVector EndPoint;
	FCollisionQueryParams CollisionParams;
	if (World == nullptr)
	{
		UE_LOG(LogTemp, Warning, TEXT("WORLD object is nullptr. Returning."));
		return;

	}
	CollisionParams.AddIgnoredActor(World->GetFirstPlayerController()->GetPawn());

	for (float Angle = -Settings.FOV / 2; Angle <= Settings.FOV / 2; Angle += Settings.AngularResolution)
	{

		if (Settings.FOV == 360.0f && Angle == Settings.FOV / 2)
		{
			continue;
		}

		float ch = FMath::Cos(FMath::DegreesToRadians(Angle));
		float sh = FMath::Sin(FMath::DegreesToRadians(Angle));

		// Simulate capturing a point at the given angle and range
		EndPoint.X = ch * Settings.Range;
		EndPoint.Y = sh * Settings.Range;
		EndPoint.Z = 40.0f; // Assuming a 2D plane for simplicity

		FVector WorldStart = GetActorLocation();
		FVector WorldEnd;
		WorldEnd.X = 

		FHitResult HitResult;


		bool bCollision = World->LineTraceSingleByChannel(HitResult, WorldStart, WorldEnd, ECC_Visibility, CollisionParams);
		//{
		Ranges.Add(HitResult.Distance);
		//DrawDebugPoint(GetWorld(), HitResult.ImpactPoint, 10.f, FColor::Red, false, 1.0f);
		//}

	}

}
*/
