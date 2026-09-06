//Flight control bindings. These belong to HeliSim rather than to any aircraft:
//they drive the flight model directly through bmkhs_fnc_inputAnalogHandler and
//bmkhs_fnc_inputNonAnalogHandler, which read them by these exact names.

#define BMKHS_ANALOG(vname, vdisplayName, vtooltip) \
class vname {\
    displayName           = vdisplayName;\
    tooltip               = vtooltip;\
    onAnalog              = __EVAL(format["['%1', _this] call bmkhs_fnc_inputAnalogHandler", #vname]);\
    analogChangeThreshold = 0.01; \
}

#define BMKHS_NONANALOG(vname, vdisplayName, vtooltip) \
class vname {\
    displayName           = vdisplayName;\
    tooltip               = vtooltip;\
    onActivate            = __EVAL(format["['%1', true]  call bmkhs_fnc_inputNonAnalogHandler", #vname]);\
    onDeactivate          = __EVAL(format["['%1', false] call bmkhs_fnc_inputNonAnalogHandler", #vname]);\
}

#define BMKHS_ACTION(vname, vdisplayName, vtooltip) class vname {    displayName           = vdisplayName;    tooltip               = vtooltip;    onActivate            = __EVAL(format["['%1', true]  call bmkhs_fnc_inputControlHandle", #vname]);    onDeactivate          = __EVAL(format["['%1', false] call bmkhs_fnc_inputControlHandle", #vname]);}

//Cockpit control macros live in their own header so a pack can take them without this
//file's class, which it would collide with when reopening CfgUserActions for its own rows.
#include "\bmkhs_helisim\controlMacros.hpp"

class CfgUserActions {
    BMKHS_ANALOG(bmkhs_cyclicForward,"Cyclic Forward","Cyclic Forward");
    BMKHS_ANALOG(bmkhs_cyclicBackward,"Cyclic Backward","Cyclic Backward");
    BMKHS_ANALOG(bmkhs_cyclicLeft,"Cyclic Left","Cyclic Left");
    BMKHS_ANALOG(bmkhs_cyclicRight,"Cyclic Right","Cyclic Right");
    BMKHS_ANALOG(bmkhs_pedalLeft,"Pedal Left","Pedal Left");
    BMKHS_ANALOG(bmkhs_pedalRight,"Pedal Right","Pedal Right");
    BMKHS_ANALOG(bmkhs_collectiveUp,"Collective Up","Collective Up");
    BMKHS_ANALOG(bmkhs_collectiveDn,"Collective Down","Collective Down");
    BMKHS_NONANALOG(bmkhs_kbCollectiveUp,"Keyboard Collective Up","Keyboard Collective Up");
    BMKHS_NONANALOG(bmkhs_kbCollectiveDn,"Keyboard Collective Down","Keyboard Collective Down");

    //Force trim / hold modes / sticky input - flight control, so HeliSim's own
    BMKHS_ACTION(bmkhs_forceTrim,"Force Trim","Hold to interrupt the hold modes, release to set new references");
    BMKHS_ACTION(bmkhs_forceTrimPanic,"Force Trim Reset","Zero the trim references and recentre the controls");
    BMKHS_ACTION(bmkhs_holdModeAltitude,"Altitude Hold","Toggle altitude hold");
    BMKHS_ACTION(bmkhs_holdModeAttitude,"Attitude Hold","Toggle attitude hold");
    BMKHS_ACTION(bmkhs_holdModesOff,"Hold Modes Off","Disengage all hold modes");
    BMKHS_ACTION(bmkhs_stickyInterrupt,"Sticky Control Interrupt","Hold to stop keyboard cyclic/pedal accumulating");
};
