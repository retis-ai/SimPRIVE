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


#include "QRCodeWrapper.h"

// Sets default values
AQRCodeWrapper::AQRCodeWrapper()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = false;
	RootComponent = CreateDefaultSubobject<USceneComponent>(TEXT("RootComponent"));

	StaticMeshComponent = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("StaticMeshComponent"));
	StaticMeshComponent->SetupAttachment(RootComponent);

	DecalComponent = CreateDefaultSubobject<UDecalComponent>(TEXT("DecalComponent"));

	static ConstructorHelpers::FObjectFinder<UStaticMesh> MeshAsset(TEXT("/SimPRIVE/Meshes/Shape_Cube.Shape_Cube"));
	static ConstructorHelpers::FObjectFinder<UMaterialInterface> DecalAsset(TEXT("/SimPRIVE/Materials/MI_QRCodeDecalMaterial.MI_QRCodeDecalMaterial"));
	if (MeshAsset.Succeeded())
	{
		StaticMeshComponent->SetStaticMesh(MeshAsset.Object);
		StaticMeshComponent->SetRelativeScale3D(FVector(0.1f, 1.0f, 1.0f));
		UE_LOG(LogTemp, Warning, TEXT("QR code Static Mesh found correctly"));
	}
	else
	{
		UE_LOG(LogTemp, Warning, TEXT("QR code Static Mesh NOT FOUND"));
	}

	if (DecalAsset.Succeeded())
	{
		DecalComponent->SetDecalMaterial(DecalAsset.Object);

		// Set the decal size
		DecalComponent->DecalSize = FVector(10.0f, 50.0f, 50.0f);

		// Attach the decal to the mesh component
		DecalComponent->SetupAttachment(StaticMeshComponent);

		// Set the relative location and rotation of the decal
		DecalComponent->SetRelativeLocation(FVector(-50.0f, 0.0f, 50.0f));
		DecalComponent->SetRelativeRotation(FRotator(180.0f, 0.0f, 0.0f));

		// Register the component
		//DecalComponent->RegisterComponent();
		UE_LOG(LogTemp, Warning, TEXT("QR Code Decal asset found correctly"));
	}
	else
	{
		UE_LOG(LogTemp, Warning, TEXT("QR code decal asset NOT FOUND"));
	}
}

// Called when the game starts or when spawned
void AQRCodeWrapper::BeginPlay()
{
	Super::BeginPlay();
	
}

// Called every frame
void AQRCodeWrapper::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

}
