//Default bindings for HeliSim's own flight control actions. Core owns the actions,
//so it owns their defaults - an aircraft is free to rebind them.
class CfgDefaultKeysPresets {
    class Arma2 {
        class Mappings {
            bmkhs_stickyInterrupt[] = {0x39};
            bmkhs_forceTrimPanic[] = {};
            bmkhs_forceTrim[] = {};
            bmkhs_holdModeAltitude[] = {};
            bmkhs_holdModeAttitude[] = {};
            bmkhs_holdModesOff[] = {};
            bmkhs_cyclicForward[] = {0x11};
            bmkhs_cyclicBackward[] = {0x1F};
            bmkhs_cyclicLeft[] = {0x1E};
            bmkhs_cyclicRight[] = {0x20};
            bmkhs_pedalLeft[] = {0x10};
            bmkhs_pedalRight[] = {0x12};
            bmkhs_collectiveUp[] = {};
            bmkhs_collectiveDn[] = {};
            bmkhs_kbCollectiveUp[] = {0x2A};
            bmkhs_kbCollectiveDn[] = {0x1D};
        };
    };
};
