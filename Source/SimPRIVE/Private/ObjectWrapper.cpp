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


#include "ObjectWrapper.h"

// Sets default values
AObjectWrapper::AObjectWrapper()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = false;
	RootComponent = CreateDefaultSubobject<USceneComponent>(TEXT("RootComponent"));

	StaticMeshComponent = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("StaticMeshComponent"));
	StaticMeshComponent->SetupAttachment(RootComponent);

	// Enable overlap events
	StaticMeshComponent->SetGenerateOverlapEvents(true);

	// Set collision enabled for query only
	StaticMeshComponent->SetCollisionEnabled(ECollisionEnabled::QueryOnly);

	// Set collision object type
	StaticMeshComponent->SetCollisionObjectType(ECollisionChannel::ECC_WorldDynamic);

	// Set collision response to channels
	StaticMeshComponent->SetCollisionResponseToAllChannels(ECR_Ignore);
	StaticMeshComponent->SetCollisionResponseToChannel(ECC_Visibility, ECR_Block);
	StaticMeshComponent->SetCollisionResponseToChannel(ECC_WorldStatic, ECR_Overlap);
	StaticMeshComponent->SetCollisionResponseToChannel(ECC_WorldDynamic, ECR_Overlap);
	StaticMeshComponent->SetCollisionResponseToChannel(ECC_Pawn, ECR_Overlap);
}

// Called when the game starts or when spawned
void AObjectWrapper::BeginPlay()
{
	Super::BeginPlay();
	
}

// Called every frame
void AObjectWrapper::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

}

void AObjectWrapper::SetupMesh(UStaticMesh* StaticMesh)
{
	if (StaticMesh)
	{
		StaticMeshComponent->SetStaticMesh(StaticMesh);
	}
}

