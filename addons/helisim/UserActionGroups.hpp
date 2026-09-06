//HeliSim's flight control actions, grouped for the controls menu.
//
//The second view of the cockpit-control row table: an aircraft includes its rows once in
//CfgUserActions to emit the classes, then again inside a group[] with these definitions to
//emit the names. Keep them adjacent to the originals - a name in a group with no matching
//class is a bind that appears in the menu and does nothing, with no build error.
//
//Core declares no group of its own: filling one would mean naming an aircraft's header.
#define QUOTE(x) #x
#undef BMKHS_CONTROL
#define BMKHS_CONTROL(cname,ptok,pnum,vdisplayName) QUOTE(bmkhs_ctrl_##cname##_##ptok)
#undef BMKHS_CONTROL_SEP
#define BMKHS_CONTROL_SEP() ,

class UserActionGroups {
    class bmkhs_flightControls {
        name = "HeliSim Flight Controls";
        group[] = {
            "bmkhs_cyclicForward",
            "bmkhs_cyclicBackward",
            "bmkhs_cyclicLeft",
            "bmkhs_cyclicRight",
            "bmkhs_pedalLeft",
            "bmkhs_pedalRight",
            "bmkhs_collectiveUp",
            "bmkhs_collectiveDn",
            "bmkhs_kbCollectiveUp",
            "bmkhs_kbCollectiveDn",
            "bmkhs_forceTrim",
            "bmkhs_forceTrimPanic",
            "bmkhs_holdModeAltitude",
            "bmkhs_holdModeAttitude",
            "bmkhs_holdModesOff",
            "bmkhs_stickyInterrupt"
        };
    };
};

class UserActionsConflictGroups {
    class bmkhs_flightControls {
        group[] = {
            "bmkhs_cyclicForward",
            "bmkhs_cyclicBackward",
            "bmkhs_cyclicLeft",
            "bmkhs_cyclicRight",
            "bmkhs_pedalLeft",
            "bmkhs_pedalRight",
            "bmkhs_collectiveUp",
            "bmkhs_collectiveDn",
            "bmkhs_kbCollectiveUp",
            "bmkhs_kbCollectiveDn",
            "bmkhs_forceTrim",
            "bmkhs_forceTrimPanic",
            "bmkhs_holdModeAltitude",
            "bmkhs_holdModeAttitude",
            "bmkhs_holdModesOff",
            "bmkhs_stickyInterrupt"
        };
    };
};
