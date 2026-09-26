// ============================================================================
// Engine Display - display definition
// IDD 5300, IDCs 5301-5399
//
// Bar controls are a fixed pool sized for BMKHS_ENGDISP_MAX_ENG engines; the
// update function positions and hides them from the BG control's ctrlPosition.
// Inner classes only; RscTitles and CfgUIGrids are opened by RscCtrlVis.hpp.
// ============================================================================

#define BMKHS_ENGDISP_MAX_ENG 4

//Tape IDC bases - one control per engine per parameter, plus the Nr tape.
//base + n addresses engine n, so the update loop is arithmetic rather than a lookup.
#define IDC_ED_TQ_FRAME   5310
#define IDC_ED_TQ_FILL    5320
#define IDC_ED_TQ_NUM     5330
#define IDC_ED_NP_FRAME   5340
#define IDC_ED_NP_FILL    5350
#define IDC_ED_NP_NUM     5360
#define IDC_ED_TGT_FRAME  5370
#define IDC_ED_TGT_FILL   5380
#define IDC_ED_TGT_NUM    5390
#define IDC_ED_ENGNUM     5400
#define IDC_ED_NG_NUM     5410
#define IDC_ED_OIL_NUM    5420
#define IDC_ED_RTG_NUM    5430
#define IDC_ED_BAND_AMBER 5440
#define IDC_ED_BAND_RED   5450

#define IDC_ED_NR_FRAME   5304
#define IDC_ED_NR_FILL    5305
#define IDC_ED_NR_NUM     5306

//Group and row labels - fixed count, positioned per frame like everything else.
#define IDC_ED_LBL_TORQUE 5307
#define IDC_ED_LBL_TGT    5308
#define IDC_ED_LBL_NR     5309
#define IDC_ED_LBL_NG     5460
#define IDC_ED_LBL_OIL    5461
#define IDC_ED_LBL_RTG    5462
#define IDC_ED_LBL_NP     5470

#define IDC_ED_ANNUN      5480

class bmkhs_engdisplay
{
    idd          = 5300;
    movingEnable = 1;
    sizeEnable   = 0;       //Move only - size is declared in config, not resized in game
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
            //Panel size, declared here only - the grid saves X/Y. 2.4 wide x 3 tall.
            //Both coefficients are in safeZoneH units and were set from measuring the
            //render, not derived - X and Y do not share a scale.
            x = "(profileNamespace getVariable ['IGUI_grid_bmkhs_engdisplay_X', safeZoneX + safeZoneW * 0.780])";
            y = "(profileNamespace getVariable ['IGUI_grid_bmkhs_engdisplay_Y', safeZoneY + safeZoneH * 0.120])";
            w = "safeZoneH * 0.232";
            h = "safeZoneH * 0.400";
        };

        //Every control below is positioned per frame by bmkhs_fnc_engDisplayUpdate, so
        //the geometry here only has to be off-screen and harmless before the first tick.
        #define ED_HIDDEN x = 0; y = 0; w = 0; h = 0

        class BMKHS_EngDisplay_TapeFrame : RscText
        {
            idc = -1;
            colorBackground[] = {0.35, 0.40, 0.35, 0.55};
            colorText[]       = {0, 0, 0, 0};
            text = "";
            ED_HIDDEN;
        };
        class BMKHS_EngDisplay_TapeFill : BMKHS_EngDisplay_TapeFrame
        {
            colorBackground[] = {0.20, 0.90, 0.20, 0.95};
        };
        class BMKHS_EngDisplay_Band : BMKHS_EngDisplay_TapeFrame
        {
            colorBackground[] = {0.95, 0.75, 0.10, 0.85};
        };
        class BMKHS_EngDisplay_Num : RscText
        {
            idc    = -1;
            colorBackground[] = {0, 0, 0, 0};
            colorText[]       = {0.80, 1.00, 0.80, 1.00};
            font   = "EtelkaMonospacePro";
            sizeEx = 0.024;
            style  = 2;
            text   = "";
            ED_HIDDEN;
        };
        class BMKHS_EngDisplay_Label : BMKHS_EngDisplay_Num
        {
            colorText[] = {0.55, 0.75, 0.55, 1.00};
            sizeEx      = 0.021;
        };
        class BMKHS_EngDisplay_LabelL : BMKHS_EngDisplay_Label
        {
            style = 0;
        };

        //Torque
        class ED_TqFrame0 : BMKHS_EngDisplay_TapeFrame { idc = 5310; };
        class ED_TqFrame1 : BMKHS_EngDisplay_TapeFrame { idc = 5311; };
        class ED_TqFrame2 : BMKHS_EngDisplay_TapeFrame { idc = 5312; };
        class ED_TqFrame3 : BMKHS_EngDisplay_TapeFrame { idc = 5313; };
        //Fills before ticks - z-order follows declaration order, and a tick has to
        //stay visible with the tape filled past it.
        class ED_TqFill0  : BMKHS_EngDisplay_TapeFill  { idc = 5320; };
        class ED_TqFill1  : BMKHS_EngDisplay_TapeFill  { idc = 5321; };
        class ED_TqFill2  : BMKHS_EngDisplay_TapeFill  { idc = 5322; };
        class ED_TqFill3  : BMKHS_EngDisplay_TapeFill  { idc = 5323; };
        class ED_TqAmber0 : BMKHS_EngDisplay_Band      { idc = 5440; };
        class ED_TqAmber1 : BMKHS_EngDisplay_Band      { idc = 5441; };
        class ED_TqAmber2 : BMKHS_EngDisplay_Band      { idc = 5442; };
        class ED_TqAmber3 : BMKHS_EngDisplay_Band      { idc = 5443; };
        class ED_TqRed0   : BMKHS_EngDisplay_Band      { idc = 5450; colorBackground[] = {0.90, 0.15, 0.15, 0.85}; };
        class ED_TqRed1   : BMKHS_EngDisplay_Band      { idc = 5451; colorBackground[] = {0.90, 0.15, 0.15, 0.85}; };
        class ED_TqRed2   : BMKHS_EngDisplay_Band      { idc = 5452; colorBackground[] = {0.90, 0.15, 0.15, 0.85}; };
        class ED_TqRed3   : BMKHS_EngDisplay_Band      { idc = 5453; colorBackground[] = {0.90, 0.15, 0.15, 0.85}; };
        class ED_TqNum0   : BMKHS_EngDisplay_Num       { idc = 5330; };
        class ED_TqNum1   : BMKHS_EngDisplay_Num       { idc = 5331; };
        class ED_TqNum2   : BMKHS_EngDisplay_Num       { idc = 5332; };
        class ED_TqNum3   : BMKHS_EngDisplay_Num       { idc = 5333; };

        //Np
        class ED_NpFrame0 : BMKHS_EngDisplay_TapeFrame { idc = 5340; };
        class ED_NpFrame1 : BMKHS_EngDisplay_TapeFrame { idc = 5341; };
        class ED_NpFrame2 : BMKHS_EngDisplay_TapeFrame { idc = 5342; };
        class ED_NpFrame3 : BMKHS_EngDisplay_TapeFrame { idc = 5343; };
        class ED_NpFill0  : BMKHS_EngDisplay_TapeFill  { idc = 5350; };
        class ED_NpFill1  : BMKHS_EngDisplay_TapeFill  { idc = 5351; };
        class ED_NpFill2  : BMKHS_EngDisplay_TapeFill  { idc = 5352; };
        class ED_NpFill3  : BMKHS_EngDisplay_TapeFill  { idc = 5353; };
        class ED_NpNum0   : BMKHS_EngDisplay_Num       { idc = 5360; };
        class ED_NpNum1   : BMKHS_EngDisplay_Num       { idc = 5361; };
        class ED_NpNum2   : BMKHS_EngDisplay_Num       { idc = 5362; };
        class ED_NpNum3   : BMKHS_EngDisplay_Num       { idc = 5363; };
        class ED_NpAmber0 : BMKHS_EngDisplay_Band      { idc = 5490; };
        class ED_NpAmber1 : BMKHS_EngDisplay_Band      { idc = 5491; };
        class ED_NpAmber2 : BMKHS_EngDisplay_Band      { idc = 5492; };
        class ED_NpAmber3 : BMKHS_EngDisplay_Band      { idc = 5493; };
        class ED_NpRed0   : BMKHS_EngDisplay_Band      { idc = 5500; colorBackground[] = {0.90, 0.15, 0.15, 0.85}; };
        class ED_NpRed1   : BMKHS_EngDisplay_Band      { idc = 5501; colorBackground[] = {0.90, 0.15, 0.15, 0.85}; };
        class ED_NpRed2   : BMKHS_EngDisplay_Band      { idc = 5502; colorBackground[] = {0.90, 0.15, 0.15, 0.85}; };
        class ED_NpRed3   : BMKHS_EngDisplay_Band      { idc = 5503; colorBackground[] = {0.90, 0.15, 0.15, 0.85}; };
        class ED_NpLbl0   : BMKHS_EngDisplay_Label     { idc = 5470; text = "NP"; };
        class ED_NpLbl1   : BMKHS_EngDisplay_Label     { idc = 5471; text = "NP"; };
        class ED_NpLbl2   : BMKHS_EngDisplay_Label     { idc = 5472; text = "NP"; };
        class ED_NpLbl3   : BMKHS_EngDisplay_Label     { idc = 5473; text = "NP"; };

        //Nr - one tape, centred among the Np tapes so a split reads instantly
        class ED_NrFrame : BMKHS_EngDisplay_TapeFrame { idc = 5304; };
        class ED_NrFill  : BMKHS_EngDisplay_TapeFill  { idc = 5305; };
        class ED_NrAmber : BMKHS_EngDisplay_Band      { idc = 5530; };
        class ED_NrRed   : BMKHS_EngDisplay_Band      { idc = 5531; colorBackground[] = {0.90, 0.15, 0.15, 0.85}; };
        class ED_NrNum   : BMKHS_EngDisplay_Num       { idc = 5306; };
        class ED_NrLbl   : BMKHS_EngDisplay_Label     { idc = 5309; text = "NR"; };

        //TGT
        class ED_TgtFrame0 : BMKHS_EngDisplay_TapeFrame { idc = 5370; };
        class ED_TgtFrame1 : BMKHS_EngDisplay_TapeFrame { idc = 5371; };
        class ED_TgtFrame2 : BMKHS_EngDisplay_TapeFrame { idc = 5372; };
        class ED_TgtFrame3 : BMKHS_EngDisplay_TapeFrame { idc = 5373; };
        class ED_TgtFill0  : BMKHS_EngDisplay_TapeFill  { idc = 5380; };
        class ED_TgtFill1  : BMKHS_EngDisplay_TapeFill  { idc = 5381; };
        class ED_TgtFill2  : BMKHS_EngDisplay_TapeFill  { idc = 5382; };
        class ED_TgtFill3  : BMKHS_EngDisplay_TapeFill  { idc = 5383; };
        class ED_TgtAmber0 : BMKHS_EngDisplay_Band      { idc = 5510; };
        class ED_TgtAmber1 : BMKHS_EngDisplay_Band      { idc = 5511; };
        class ED_TgtAmber2 : BMKHS_EngDisplay_Band      { idc = 5512; };
        class ED_TgtAmber3 : BMKHS_EngDisplay_Band      { idc = 5513; };
        class ED_TgtRed0   : BMKHS_EngDisplay_Band      { idc = 5520; colorBackground[] = {0.90, 0.15, 0.15, 0.85}; };
        class ED_TgtRed1   : BMKHS_EngDisplay_Band      { idc = 5521; colorBackground[] = {0.90, 0.15, 0.15, 0.85}; };
        class ED_TgtRed2   : BMKHS_EngDisplay_Band      { idc = 5522; colorBackground[] = {0.90, 0.15, 0.15, 0.85}; };
        class ED_TgtRed3   : BMKHS_EngDisplay_Band      { idc = 5523; colorBackground[] = {0.90, 0.15, 0.15, 0.85}; };
        class ED_TgtNum0   : BMKHS_EngDisplay_Num       { idc = 5390; };
        class ED_TgtNum1   : BMKHS_EngDisplay_Num       { idc = 5391; };
        class ED_TgtNum2   : BMKHS_EngDisplay_Num       { idc = 5392; };
        class ED_TgtNum3   : BMKHS_EngDisplay_Num       { idc = 5393; };

        //Engine number - one above each tape in every group, and once more over the
        //digital rows, so no column is unlabelled.
        class ED_EngNum0 : BMKHS_EngDisplay_Label { idc = 5400; };
        class ED_EngNum1 : BMKHS_EngDisplay_Label { idc = 5401; };
        class ED_EngNum2 : BMKHS_EngDisplay_Label { idc = 5402; };
        class ED_EngNum3 : BMKHS_EngDisplay_Label { idc = 5403; };
        class ED_TgtEngNum0 : BMKHS_EngDisplay_Label { idc = 5540; };
        class ED_TgtEngNum1 : BMKHS_EngDisplay_Label { idc = 5541; };
        class ED_TgtEngNum2 : BMKHS_EngDisplay_Label { idc = 5542; };
        class ED_TgtEngNum3 : BMKHS_EngDisplay_Label { idc = 5543; };
        class ED_RowEngNum0 : BMKHS_EngDisplay_Label { idc = 5550; };
        class ED_RowEngNum1 : BMKHS_EngDisplay_Label { idc = 5551; };
        class ED_RowEngNum2 : BMKHS_EngDisplay_Label { idc = 5552; };
        class ED_RowEngNum3 : BMKHS_EngDisplay_Label { idc = 5553; };

        //Digital rows
        class ED_NgNum0  : BMKHS_EngDisplay_Num { idc = 5410; };
        class ED_NgNum1  : BMKHS_EngDisplay_Num { idc = 5411; };
        class ED_NgNum2  : BMKHS_EngDisplay_Num { idc = 5412; };
        class ED_NgNum3  : BMKHS_EngDisplay_Num { idc = 5413; };
        class ED_OilNum0 : BMKHS_EngDisplay_Num { idc = 5420; };
        class ED_OilNum1 : BMKHS_EngDisplay_Num { idc = 5421; };
        class ED_OilNum2 : BMKHS_EngDisplay_Num { idc = 5422; };
        class ED_OilNum3 : BMKHS_EngDisplay_Num { idc = 5423; };
        class ED_RtgNum0 : BMKHS_EngDisplay_Num { idc = 5430; };
        class ED_RtgNum1 : BMKHS_EngDisplay_Num { idc = 5431; };
        class ED_RtgNum2 : BMKHS_EngDisplay_Num { idc = 5432; };
        class ED_RtgNum3 : BMKHS_EngDisplay_Num { idc = 5433; };

        class ED_LblTorque : BMKHS_EngDisplay_Label  { idc = 5307; text = "TORQUE"; };
        class ED_LblTgt    : BMKHS_EngDisplay_Label  { idc = 5308; text = "TGT"; };
        class ED_LblTach   : BMKHS_EngDisplay_Label  { idc = 5560; text = "NP / NR"; };
        class ED_LblNg     : BMKHS_EngDisplay_LabelL { idc = 5460; text = "NG"; };
        class ED_LblOil    : BMKHS_EngDisplay_LabelL { idc = 5461; text = "OIL"; };
        class ED_LblRtg    : BMKHS_EngDisplay_LabelL { idc = 5462; text = "RTG"; };

        //Warnings, cautions and advisories. Structured text so each line carries its
        //own colour; only active lines are rendered.
        class ED_Annun : RscStructuredText
        {
            idc  = 5480;
            text = "";
            size = "safeZoneH * 0.020";
            colorBackground[] = {0, 0, 0, 0.35};
            ED_HIDDEN;
        };
    };
};
