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
#include "ROSIntegrationGameInstance.h"


#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"
#include "Interfaces/IPluginManager.h"


#include "SimPRIVEGameInstance.generated.h"

/**
 * 
 */
UCLASS()
class SIMPRIVE_API USimPRIVEGameInstance : public UROSIntegrationGameInstance
{
	GENERATED_BODY()

public:
	USimPRIVEGameInstance();
	virtual void Init() override;


	bool ReadIPConfig(FString& OutIP, int32& OutPort);

	
};
