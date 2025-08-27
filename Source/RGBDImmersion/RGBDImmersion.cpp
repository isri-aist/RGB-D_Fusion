#include "RGBDImmersion.h"
#include <cstdlib>

IMPLEMENT_PRIMARY_GAME_MODULE(FRGBDImmersionGameModule, RGBDImmersion, "RGBDImmersion");

void FRGBDImmersionGameModule::StartupModule()
{
    // Forcer ROS_DOMAIN_ID dès le chargement du module
    setenv("ROS_DOMAIN_ID", "20", 1);

    UE_LOG(LogTemp, Warning, TEXT("ROS_DOMAIN_ID défini à 20 dans StartupModule"));
}

