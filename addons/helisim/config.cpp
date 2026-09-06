class CfgPatches
{
    class bmkhs_helisim
    {
        units[] = {};
        author = "BradMick";
        weapons[] = {};
        requiredVersion = 2.10;
        requiredAddons[] = {"A3_Air_F_Beta", "cba_main", "cba_xeh"};
        #include "version.hpp"
    };
};

#include "CfgFunctions.hpp"
#include "CfgUserActions.hpp"
#include "CfgDefaultKeysPresets.hpp"
#include "UserActionGroups.hpp"
#include "extendedEventHandlers.hpp"
#include "ui\RscCtrlVis.hpp"
