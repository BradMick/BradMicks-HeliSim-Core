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

    //The control input visualiser is Core's display of Core's own inputs, so its bind is too
    BMKHS_ACTION(bmkhs_ctrlVisToggle,"Control Input Visualiser: Toggle","Show or hide the control input visualiser");

    //Flight director - modes, then each target's step, sync and value. Act only on an aircraft
    //that declares FMC >> FlightDirector.
    BMKHS_ACTION(bmkhs_fdRalt,"FD Radar Altitude","Toggle the flight director radar altitude mode");
    BMKHS_ACTION(bmkhs_fdAlt,"FD Barometric Altitude","Toggle the flight director barometric altitude mode");
    BMKHS_ACTION(bmkhs_fdAltp,"FD Altitude Preselect","Toggle the flight director altitude preselect mode");
    BMKHS_ACTION(bmkhs_fdIas,"FD Airspeed","Toggle the flight director airspeed mode");
    BMKHS_ACTION(bmkhs_fdHdg,"FD Heading","Toggle the flight director heading mode");
    BMKHS_ACTION(bmkhs_fdNav,"FD Nav Coupled","Toggle the flight director nav coupled mode");
    BMKHS_ACTION(bmkhs_fdHvr,"FD Hover","Toggle the flight director hover mode");
    BMKHS_ACTION(bmkhs_fdRaltUp,"FD Radar Altitude Up","Step the radar altitude target up");
    BMKHS_ACTION(bmkhs_fdRaltDn,"FD Radar Altitude Down","Step the radar altitude target down");
    BMKHS_ACTION(bmkhs_fdRaltSync,"FD Radar Altitude Sync","Set the radar altitude target to the aircraft's");
    BMKHS_ANALOG(bmkhs_fdRaltTarget,"FD Radar Altitude Target","Set the radar altitude target across its range");
    BMKHS_ACTION(bmkhs_fdAltUp,"FD Barometric Altitude Up","Step the barometric altitude target up");
    BMKHS_ACTION(bmkhs_fdAltDn,"FD Barometric Altitude Down","Step the barometric altitude target down");
    BMKHS_ACTION(bmkhs_fdAltSync,"FD Barometric Altitude Sync","Set the barometric altitude target to the aircraft's");
    BMKHS_ANALOG(bmkhs_fdAltTarget,"FD Barometric Altitude Target","Set the barometric altitude target across its range");
    BMKHS_ACTION(bmkhs_fdAltpUp,"FD Altitude Preselect Up","Step the altitude preselect target up");
    BMKHS_ACTION(bmkhs_fdAltpDn,"FD Altitude Preselect Down","Step the altitude preselect target down");
    BMKHS_ACTION(bmkhs_fdAltpSync,"FD Altitude Preselect Sync","Set the altitude preselect target to the aircraft's");
    BMKHS_ANALOG(bmkhs_fdAltpTarget,"FD Altitude Preselect Target","Set the altitude preselect target across its range");
    BMKHS_ACTION(bmkhs_fdIasUp,"FD Airspeed Up","Step the airspeed target up");
    BMKHS_ACTION(bmkhs_fdIasDn,"FD Airspeed Down","Step the airspeed target down");
    BMKHS_ACTION(bmkhs_fdIasSync,"FD Airspeed Sync","Set the airspeed target to the aircraft's");
    BMKHS_ANALOG(bmkhs_fdIasTarget,"FD Airspeed Target","Set the airspeed target across its range");
    BMKHS_ACTION(bmkhs_fdHdgUp,"FD Heading Up","Step the heading target up");
    BMKHS_ACTION(bmkhs_fdHdgDn,"FD Heading Down","Step the heading target down");
    BMKHS_ACTION(bmkhs_fdHdgSync,"FD Heading Sync","Set the heading target to the aircraft's");
    BMKHS_ANALOG(bmkhs_fdHdgTarget,"FD Heading Target","Set the heading target across its range");
};
