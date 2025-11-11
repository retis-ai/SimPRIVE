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


#include "SpawningManager.h"

// Sets default values
ASpawningManager::ASpawningManager()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = false;
	RootComponent = CreateDefaultSubobject<USceneComponent>(TEXT("RootComponent"));

}

// Called when the game starts or when spawned
void ASpawningManager::BeginPlay()
{
	Super::BeginPlay();
	
}

// Called every frame
void ASpawningManager::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

}

void ASpawningManager::AddNewSpline()
{
    int i = Paths.Num();
    FString SplineName = FString::Printf(TEXT("SplineComponent_%d"), i);
    USplineComponent* path = NewObject<USplineComponent>(this, *SplineName);


    path->SetMobility(EComponentMobility::Movable);
    path->AttachToComponent(RootComponent, FAttachmentTransformRules::KeepRelativeTransform);
    path->RegisterComponent();

    path->ClearSplinePoints();
    FVector PointLocation = GetActorLocation();
    path->AddSplinePoint(PointLocation, ESplineCoordinateSpace::World, true);
    PointLocation.X += 1000.0;
    path->AddSplinePoint(PointLocation, ESplineCoordinateSpace::World, true);
    path->UpdateSpline();
    Paths.Add(path);

    RegisterAllComponents();

}

void ASpawningManager::SpawnPedestrian()
{
    UE_LOG(LogTemp, Warning, TEXT("Trying to spawn a pedestrian"));

    if (SkeletalMeshLibrary.IsEmpty() || AnimationLibrary.IsEmpty() || Paths.IsEmpty())
    {
        UE_LOG(LogTemp, Warning, TEXT("One of the libraries is empty!"));
        return;
    }

    FActorSpawnParameters SpawnParameters;
    SpawnParameters.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;

    FVector RandomLocation(0.0f, 0.0f, 1000.0f);
    FRotator RandomRotation(0.0f, 0.0f, 0.0f);
    //GetRandomSpawnPoint(RandomLocation, RandomRotation);


    int RandomMeshIdx = FMath::RandRange(0, SkeletalMeshLibrary.Num() - 1);
    int RandomAnimIdx = FMath::RandRange(0, AnimationLibrary.Num() - 1);
    int RandomPathIdx = FMath::RandRange(0, Paths.Num() - 1);

    if (!GetWorld())
    {
        UE_LOG(LogTemp, Error, TEXT("Invalid world context!"));
        return;
    }

    APedestrianWrapper* Ped1 = GetWorld()->SpawnActorDeferred<APedestrianWrapper>(APedestrianWrapper::StaticClass(), FTransform(), nullptr, nullptr, ESpawnActorCollisionHandlingMethod::AlwaysSpawn);
    //APedestrianWrapper* Ped1 = GetWorld()->SpawnActor<APedestrianWrapper>(APedestrianWrapper::StaticClass(), RandomLocation, RandomRotation, SpawnParameters); // Paths[0], SkeletalMeshLibrary[0], AnimationLibrary[0]);
    if (Ped1)
    {
        Ped1->SetAnimation(AnimationLibrary[RandomAnimIdx]);
        Ped1->SetSkeletalMesh(SkeletalMeshLibrary[RandomMeshIdx]);
        USplineComponent* Spline = Paths[RandomPathIdx];
        if (Spline)
        {
            Ped1->SetPath(Spline, FMath::RandRange(0.0f, Spline->GetSplineLength()));
            UE_LOG(LogTemp, Warning, TEXT("Spawned!"));
        }
        else
        {
            UE_LOG(LogTemp, Warning, TEXT("Spline not found!"));

        }
        GetFreeSpawnPoint(RandomLocation, RandomRotation);

        Ped1->FinishSpawning(FTransform(RandomRotation, RandomLocation, FVector(1.0f, 1.0f, 1.0f)));



        Pedestrians.Add(Ped1);
    }

}

void ASpawningManager::SpawnObject()
{

    if (StaticMeshLibrary.IsEmpty())
    {
        UE_LOG(LogTemp, Warning, TEXT("Static mesh library is empty!"));
        return;
    }

    FVector RandomLocation;
    FRotator RandomRotation;
    //GetRandomSpawnPoint(RandomLocation, RandomRotation);
    GetFreeSpawnPoint(RandomLocation, RandomRotation);
    RandomLocation.Z += 100.0f;
    FVector StartLocation = RandomLocation;
    FVector EndLocation = StartLocation - FVector(0.0f, 0.0f, 300.0f); // Trace downwards

    FHitResult HitResult;
    FCollisionQueryParams QueryParams;
    QueryParams.AddIgnoredActor(this); // Ignore the actor performing the trace

    bool bHit = GetWorld()->LineTraceSingleByChannel(HitResult, StartLocation, EndLocation, ECC_Visibility, QueryParams);

    FVector SpawnLocation = RandomLocation;
    if (bHit)
    {
        SpawnLocation = HitResult.Location;
    }


    float Scale = 1.0f;
    FTransform SpawnTransform(RandomRotation, SpawnLocation, FVector(Scale, Scale, Scale));
    int RandomMeshIdx = FMath::RandRange(0, StaticMeshLibrary.Num() - 1);
    AObjectWrapper* Obj = GetWorld()->SpawnActorDeferred<AObjectWrapper>(AObjectWrapper::StaticClass(), SpawnTransform, nullptr, nullptr, ESpawnActorCollisionHandlingMethod::AlwaysSpawn);

    if (Obj)
    {
        Obj->SetupMesh(StaticMeshLibrary[RandomMeshIdx]);
        Obj->FinishSpawning(SpawnTransform);

        Objects.Add(Obj);
    }
}

void ASpawningManager::SpawnQRCodeObject()
{
    FVector RandomLocation;
    FRotator RandomRotation;
    //GetRandomSpawnPoint(RandomLocation, RandomRotation);
    //GetFreeSpawnPoint(RandomLocation, RandomRotation);
    GetQRCodeSpawnPoint(RandomLocation, RandomRotation);
    RandomLocation.Z += 200.0f;
    //RandomRotation.Roll += 90.0f;
    FVector StartLocation = RandomLocation;
    FVector EndLocation = StartLocation - FVector(0.0f, 0.0f, 300.0f); // Trace downwards

    FHitResult HitResult;
    FCollisionQueryParams QueryParams;
    QueryParams.AddIgnoredActor(this); // Ignore the actor performing the trace

    bool bHit = GetWorld()->LineTraceSingleByChannel(HitResult, StartLocation, EndLocation, ECC_Visibility, QueryParams);

    FVector SpawnLocation = RandomLocation;
    UE_LOG(LogTemp, Warning, TEXT("Spawn location QR code z %f"), SpawnLocation.Z);
    if (bHit)
    {
        SpawnLocation = HitResult.Location;
        SpawnLocation.Z += 50.0f;
    }



    UE_LOG(LogTemp, Warning, TEXT("Spawn location QR code z %f"), SpawnLocation.Z);
    FTransform SpawnTransform(RandomRotation, SpawnLocation, FVector(1.0f, 1.0f, 1.0f));
    AQRCodeWrapper* QRCode = GetWorld()->SpawnActorDeferred<AQRCodeWrapper>(AQRCodeWrapper::StaticClass(), SpawnTransform, nullptr, nullptr, ESpawnActorCollisionHandlingMethod::AlwaysSpawn);

    if (QRCode)
    {
        QRCode->FinishSpawning(SpawnTransform);

        QRCodes.Add(QRCode);
    }
}

void ASpawningManager::GetRandomSpawnPoint(FVector& Location, FRotator& Rotation)
{
    if (Paths.Num() > 0)
    {
        int RandomSplineIdx = FMath::RandRange(0, 2);
        auto spline = Paths[RandomSplineIdx];

        // todo check occupancy
        float RandomDist = FMath::RandRange(0.0f, spline->GetSplineLength());

        Location = spline->GetLocationAtDistanceAlongSpline(RandomDist, ESplineCoordinateSpace::World);
        Rotation = spline->GetRotationAtDistanceAlongSpline(RandomDist, ESplineCoordinateSpace::World);
    }
    else
    {
        UE_LOG(LogTemp, Warning, TEXT("No spline was defined! First add some splines."));
        Location = FVector::ZeroVector;
        Rotation = FRotator::ZeroRotator;

    }
}

void ASpawningManager::GetFreeSpawnPoint(FVector& Location, FRotator& Rotation)
{
    if (Paths.Num() > 0)
    {
        int RandomSplineIdx = FMath::RandRange(0, Paths.Num()-1);
        //if (FMath::RandRange(0, 1) < 0.5)
        //    RandomSplineIdx = 2;
        //float CumulativeProbability = 0.0f;
        //float RandomValue = FMath::FRand();

        auto spline = Paths[RandomSplineIdx];

        // todo check occupancy
        float RandomValue = FMath::FRand();
        float CumulativeProbability = 0.0f;
        int32 SelectedSegmentIndex = 0;
        int Length = FMath::Floor(spline->GetSplineLength() / 100.0f);
        TArray<float>& Probabilities = DistanceSpawnProbabilities[RandomSplineIdx];
        int NumSegments = Probabilities.Num();

        //UE_LOG(LogTemp, Warning, TEXT("Spline: %d"), RandomSplineIdx);


        for (int32 i = 0; i < NumSegments; ++i)
        {
            CumulativeProbability += Probabilities[i];
            //UE_LOG(LogTemp, Warning, TEXT("val: %f, IDx: %d, prob %f"), RandomValue, i, Probabilities[i]);

            if (RandomValue <= CumulativeProbability)
            {
                SelectedSegmentIndex = i;
                break;
            }
        }

        float FreeDist = SelectedSegmentIndex * 1.0f;

        Location = spline->GetLocationAtDistanceAlongSpline(FreeDist * 100.f, ESplineCoordinateSpace::World);
        Rotation = spline->GetRotationAtDistanceAlongSpline(FreeDist * 100.f, ESplineCoordinateSpace::World);

        // Update the probability distribution (set the selected segment's probability to 0 and normalize)
        Probabilities[SelectedSegmentIndex] = 0.0f;
        float TotalProbability = 0.0f;
        for (float Probability : Probabilities)
        {
            TotalProbability += Probability;
        }
        for (float& Probability : Probabilities)
        {
            Probability /= TotalProbability;
            //UE_LOG(LogTemp, Warning, TEXT("Updated prob to %f"), Probability);

        }
    }
    else
    {
        UE_LOG(LogTemp, Warning, TEXT("No spline was defined! First add some splines."));
        Location = FVector::ZeroVector;
        Rotation = FRotator::ZeroRotator;

    }
}

void ASpawningManager::GetRoverSpawnPoint(FVector& Location, FRotator& Rotation)
{
    if (Paths.Num() > 0)
    {
        int RandomSplineIdx = FMath::RandRange(0, Paths.Num() - 1);
        auto Spline = Paths[RandomSplineIdx];

        //Location = spline->GetLocationAtDistanceAlongSpline(-100.0f, ESplineCoordinateSpace::World);
        // Get the first and second points on the spline
        FVector FirstPoint = Spline->GetLocationAtSplinePoint(0, ESplineCoordinateSpace::World);
        FVector SecondPoint = Spline->GetLocationAtSplinePoint(1, ESplineCoordinateSpace::World);

        // Calculate the direction vector from the first point to the second point
        FVector Direction = (SecondPoint - FirstPoint).GetSafeNormal();

        // Move 1 meter in the opposite direction
        Location = FirstPoint - Direction * 200.f;

        Rotation = Spline->GetRotationAtDistanceAlongSpline(-100.0f, ESplineCoordinateSpace::World);
    }
    else
    {
        UE_LOG(LogTemp, Warning, TEXT("No spline was defined! First add some splines."));
        Location = FVector::ZeroVector;
        Rotation = FRotator::ZeroRotator;

    }
}

void ASpawningManager::GetQRCodeSpawnPoint(FVector& Location, FRotator& Rotation)
{
    if (Paths.Num() > 0)
    {
        int RandomSplineIdx = FMath::RandRange(0, Paths.Num()-1);
        auto spline = Paths[RandomSplineIdx];

        Location = spline->GetLocationAtDistanceAlongSpline(spline->GetSplineLength(), ESplineCoordinateSpace::World);
        Rotation = spline->GetRotationAtDistanceAlongSpline(spline->GetSplineLength(), ESplineCoordinateSpace::World);
    }
    else
    {
        UE_LOG(LogTemp, Warning, TEXT("No spline was defined! First add some splines."));
        Location = FVector::ZeroVector;
        Rotation = FRotator::ZeroRotator;

    }
}

void ASpawningManager::ResetSpawnProbabilities()
{
    SplineSpawnProbabilities.Empty();
    for (int i = 0; i < Paths.Num(); i++)
    {
        //if (i == 1)
        //{
            //SplineSpawnProbabilities.Add(0.0f);
        //}
        //else
        //{
            //SplineSpawnProbabilities.Add(1.0f / (Paths.Num() - 1));
        //}
        SplineSpawnProbabilities.Add(1.0f / Paths.Num());
    }

    DistanceSpawnProbabilities.Empty();
    for (int i = 0; i < Paths.Num(); i++)
    {
        TArray<float> SplineProb;
        int Length = FMath::Floor(Paths[i]->GetSplineLength() / 100.0f);
        for (int d = 0; d < Length; d++)
        {
            if (d < 3)
            {
                SplineProb.Add(0.0f);
            }
            else
            {
                SplineProb.Add(1.0f / Length);
            }
            //UE_LOG(LogTemp, Warning, TEXT("Spline: %d, idx %d, prob %f"), i, d, SplineProb.Last());
        }
        SplineProb[SplineProb.Num() - 1] = 0.0f;
        DistanceSpawnProbabilities.Add(SplineProb);
    }
}

void ASpawningManager::DeleteSpawnedObjects()
{
}
