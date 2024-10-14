// Copyright Epic Games, Inc. All Rights Reserved.

#include "testButtonCommands.h"

#define LOCTEXT_NAMESPACE "FtestButtonModule"

void FtestButtonCommands::RegisterCommands()
{
	UI_COMMAND(PluginAction, "newton", "newton editor updates", EUserInterfaceActionType::Button, FInputChord());
}

#undef LOCTEXT_NAMESPACE
