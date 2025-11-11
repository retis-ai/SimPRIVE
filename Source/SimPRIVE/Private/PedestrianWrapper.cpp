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


#include "PedestrianWrapper.h"

// Sets default values
APedestrianWrapper::APedestrianWrapper()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = false;

    RootComponent = CreateDefaultSubobject<USceneComponent>(TEXT("RootComponent"));
    //RootComponent = SplineComponent;

    SkeletalMeshComponent = CreateDefaultSubobject<USkeletalMeshComponent>(TEXT("SkeletalMeshComponent"));
    SkeletalMeshComponent->SetupAttachment(RootComponent);

    SplineComponent = CreateDefaultSubobject<USplineComponent>(TEXT("SplineComponent"));
    SplineComponent->SetupAttachment(RootComponent);

}

// Called when the game starts or when spawned
void APedestrianWrapper::BeginPlay()
{
	Super::BeginPlay();
	

    if (SkeletalMeshComponent && AnimationToPlay && SplineComponent)
    {

        // Set the animation mode to use a single animation asset
        SkeletalMeshComponent->SetAnimationMode(EAnimationMode::AnimationSingleNode);

        // Set the animation to play
        SkeletalMeshComponent->SetAnimation(AnimationToPlay);

        // Optionally, start playing the animation
        SkeletalMeshComponent->Play(true); // true for looping
        UE_LOG(LogTemp, Warning, TEXT("Started internal animation!"));


        // Enable overlap events
        SkeletalMeshComponent->SetGenerateOverlapEvents(true);

        // Set collision enabled for query only
        SkeletalMeshComponent->SetCollisionEnabled(ECollisionEnabled::QueryOnly);

        // Set collision object type
        SkeletalMeshComponent->SetCollisionObjectType(ECollisionChannel::ECC_WorldDynamic);

        // Set collision response to channels
        SkeletalMeshComponent->SetCollisionResponseToAllChannels(ECR_Ignore);
        SkeletalMeshComponent->SetCollisionResponseToChannel(ECC_Visibility, ECR_Block);
        SkeletalMeshComponent->SetCollisionResponseToChannel(ECC_WorldStatic, ECR_Overlap);
        SkeletalMeshComponent->SetCollisionResponseToChannel(ECC_WorldDynamic, ECR_Overlap);
        SkeletalMeshComponent->SetCollisionResponseToChannel(ECC_Pawn, ECR_Overlap);
    }
}

// Called every frame
void APedestrianWrapper::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

}


void APedestrianWrapper::SetSkeletalMesh(USkeletalMesh* SkeletalMesh)
{
    if (SkeletalMesh)
    {
        SkeletalMeshComponent->SetSkeletalMesh(SkeletalMesh);
        UE_LOG(LogTemp, Warning, TEXT("Skeletal Mesh set!"));
    }
}

void APedestrianWrapper::SetAnimation(UAnimSequence* Animation)
{
    if (Animation)
    {
        AnimationToPlay = Animation;
        UE_LOG(LogTemp, Warning, TEXT("Animation set!"));
    }
}

void APedestrianWrapper::SetPath(USplineComponent* Spline, const float Distance)
{
    if (Spline)
    {
        SplineComponent = Spline;
        //RootComponent = SplineComponent;

        CurrDistance = FMath::Fmod(Distance, SplineComponent->GetSplineLength());

        //UE_LOG(LogTemp, Warning, TEXT("***SETTING Distance: %f, %f / %f"), CurrDistance, Distance, SplineComponent->GetSplineLength());
        //UE_LOG(LogTemp, Warning, TEXT("Pedestrian path set!"));
    }
}

