// ============================================================================
// Engine Display - display definition
// IDD 5300, IDCs 5300-5310
//
// Shown only when the aircraft runs with useSystems = 0, where nothing is
// feeding the engine data to a cockpit display. Inner classes only; RscTitles
// and CfgUIGrids are opened by RscCtrlVis.hpp.
// ============================================================================

class bmkhs_engdisplay
{
    idd          = 5300;
    movingEnable = 1;
    sizeEnable   = 1;
    duration     = 99999;
    fadein       = 0;
    fadeout      = 0;
    name         = "bmkhs_engdisplay";
    onLoad       = "uiNameSpace setVariable ['bmkhs_engdisplay', _this select 0];";

    class controls
    {
        class BMKHS_EngDisplay_BG : RscText
        {
            idc = 5301;
            colorBackground[] = {0.0, 0.0, 0.0, 0.75};
            colorText[]       = {0, 0, 0, 0};
            text = "";
            x = "(profileNamespace getVariable ['IGUI_grid_bmkhs_engdisplay_X', safeZoneX + safeZoneW * 0.780])";
            y = "(profileNamespace getVariable ['IGUI_grid_bmkhs_engdisplay_Y', safeZoneY + safeZoneH * 0.180])";
            w = "(profileNamespace getVariable ['IGUI_grid_bmkhs_engdisplay_W', safeZoneH * 0.200])";
            h = "(profileNamespace getVariable ['IGUI_grid_bmkhs_engdisplay_H', safeZoneH * 0.150])";
        };

        class BMKHS_EngDisplay_DragBar : RscText
        {
            idc  = 5302;
            colorBackground[] = {0.05, 0.20, 0.05, 0.90};
            colorText[]       = {0.70, 1.00, 0.70, 1.00};
            text    = "Engine";
            font    = "PuristaMedium";
            sizeEx  = 0.028;
            style   = 2;
            x = "(profileNamespace getVariable ['IGUI_grid_bmkhs_engdisplay_X', safeZoneX + safeZoneW * 0.780])";
            y = "(profileNamespace getVariable ['IGUI_grid_bmkhs_engdisplay_Y', safeZoneY + safeZoneH * 0.180])";
            w = "(profileNamespace getVariable ['IGUI_grid_bmkhs_engdisplay_W', safeZoneH * 0.200])";
            h = "safeZoneH * 0.030";
        };

        class BMKHS_EngDisplay_Text : RscStructuredText
        {
            idc  = 5303;
            text = "";
            size = "safeZoneH * 0.020";
            colorBackground[] = {0, 0, 0, 0};
            x = "(profileNamespace getVariable ['IGUI_grid_bmkhs_engdisplay_X', safeZoneX + safeZoneW * 0.780]) + safeZoneH * 0.008";
            y = "(profileNamespace getVariable ['IGUI_grid_bmkhs_engdisplay_Y', safeZoneY + safeZoneH * 0.180]) + safeZoneH * 0.034";
            w = "(profileNamespace getVariable ['IGUI_grid_bmkhs_engdisplay_W', safeZoneH * 0.200]) - safeZoneH * 0.016";
            h = "(profileNamespace getVariable ['IGUI_grid_bmkhs_engdisplay_H', safeZoneH * 0.150]) - safeZoneH * 0.042";
        };
    };
};
