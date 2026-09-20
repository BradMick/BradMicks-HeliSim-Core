// ============================================================================
// Flight Model Debug Readout - display definition
// IDD 5200, IDCs 5200-5210
//
// This file holds ONLY the inner classes. RscTitles and CfgUIGrids may each be
// declared once, and RscCtrlVis.hpp already opens them, so this is included from
// inside those blocks rather than opening its own.
//
// A plain structured-text panel. fn_fmDebugUpdate writes the text each frame;
// nothing is repositioned at runtime, so the layout lives entirely in the
// profileNamespace keys below and the Arma layout editor can move/resize it.
// ============================================================================

class bmkhs_fmdebug
{
    idd          = 5200;
    movingEnable = 1;       // Non-blocking: the player keeps game/vehicle inputs
    sizeEnable   = 1;       // Resizable in the Arma layout editor
    duration     = 99999;
    fadein       = 0;
    fadeout      = 0;
    name         = "bmkhs_fmdebug";
    onLoad       = "uiNameSpace setVariable ['bmkhs_fmdebug', _this select 0];";

    class controls
    {
        // Background. Position is read from profileNamespace so the layout
        // editor (CfgUIGrids) can save/restore it.
        class BMKHS_FmDebug_BG : RscText
        {
            idc = 5201;
            colorBackground[] = {0.0, 0.0, 0.0, 0.75};
            colorText[]       = {0, 0, 0, 0};
            text = "";
            x = "(profileNamespace getVariable ['IGUI_grid_bmkhs_fmdebug_X', safeZoneX + safeZoneW * 0.010])";
            y = "(profileNamespace getVariable ['IGUI_grid_bmkhs_fmdebug_Y', safeZoneY + safeZoneH * 0.080])";
            w = "(profileNamespace getVariable ['IGUI_grid_bmkhs_fmdebug_W', safeZoneH * 0.520])";
            h = "(profileNamespace getVariable ['IGUI_grid_bmkhs_fmdebug_H', safeZoneH * 0.640])";
        };

        // Title bar
        class BMKHS_FmDebug_DragBar : RscText
        {
            idc  = 5202;
            colorBackground[] = {0.05, 0.20, 0.05, 0.90};
            colorText[]       = {0.70, 1.00, 0.70, 1.00};
            text    = "Flight Model - Forces";
            font    = "PuristaMedium";
            sizeEx  = 0.028;
            style   = 2;    // center
            x = "(profileNamespace getVariable ['IGUI_grid_bmkhs_fmdebug_X', safeZoneX + safeZoneW * 0.010])";
            y = "(profileNamespace getVariable ['IGUI_grid_bmkhs_fmdebug_Y', safeZoneY + safeZoneH * 0.080])";
            w = "(profileNamespace getVariable ['IGUI_grid_bmkhs_fmdebug_W', safeZoneH * 0.520])";
            h = "safeZoneH * 0.030";
        };

        // The readout itself.
        class BMKHS_FmDebug_Text : RscStructuredText
        {
            idc  = 5203;
            text = "";
            size = "safeZoneH * 0.020";
            colorBackground[] = {0, 0, 0, 0};
            x = "(profileNamespace getVariable ['IGUI_grid_bmkhs_fmdebug_X', safeZoneX + safeZoneW * 0.010]) + safeZoneH * 0.008";
            y = "(profileNamespace getVariable ['IGUI_grid_bmkhs_fmdebug_Y', safeZoneY + safeZoneH * 0.080]) + safeZoneH * 0.034";
            w = "(profileNamespace getVariable ['IGUI_grid_bmkhs_fmdebug_W', safeZoneH * 0.520]) - safeZoneH * 0.016";
            h = "(profileNamespace getVariable ['IGUI_grid_bmkhs_fmdebug_H', safeZoneH * 0.640]) - safeZoneH * 0.042";
        };
    };
};
