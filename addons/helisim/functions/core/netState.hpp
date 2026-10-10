//What the owner of an aircraft sends everyone else 10 times a second, packed into the one
//variable bmkhs_netState by fn_coreNetSend and unpacked by fn_coreNetReceive. It is what the
//crew stations display and everything the next owner needs to carry on from where this one
//left off - the engines' running state frame to frame, not just what the gauges show - so a
//change of controls picks the model up mid-flight rather than starting it cold.
//
//Latches and switch positions are not here: they are published when they change. The governor
//PIDs travel alongside, as their integral and last error only - the gains come from config.
//
//An aircraft adds values it computes itself - Core never names them - with netStateVars[] in its
//BMKHS_HeliSim config; fn_coreConfig appends them to this list as bmkhs_netStateVars.
#define NET_STATE_VARS [ \
    "bmkhs_barAlt", \
    "bmkhs_fat", \
    "bmkhs_radAlt", \
    "bmkhs_windSpeed", \
    "bmkhs_windDirFrom", \
    "bmkhs_gwt", \
    "bmkhs_cg", \
    "bmkhs_aero_beta_deg", \
    "bmkhs_aero_beta_g", \
    "bmkhs_aero_beta_g_prev", \
    "bmkhs_forceTrimPosPitch", \
    "bmkhs_forceTrimPosRoll", \
    "bmkhs_forceTrimPosYaw", \
    "bmkhs_autoPedalHdg", \
    "bmkhs_apuRpm_pct", \
    "bmkhs_collectiveOutput", \
    "bmkhs_engState", \
    "bmkhs_engFuelFlow", \
    "bmkhs_engPctNg", \
    "bmkhs_engNp", \
    "bmkhs_engPctNp", \
    "bmkhs_engPctTq", \
    "bmkhs_engOutputTq", \
    "bmkhs_engClutch", \
    "bmkhs_engTgt", \
    "bmkhs_engOilPsi", \
    "bmkhs_engOilHealth", \
    "bmkhs_engLimitTimers", \
    "bmkhs_engTqTimer", \
    "bmkhs_engResidualHeat", \
    "bmkhs_engPrevLever", \
    "bmkhs_engLeverSched", \
    "bmkhs_engNpRef", \
    "bmkhs_engLimFuel", \
    "bmkhs_engMinFuel", \
    "bmkhs_engIdleSince", \
    "bmkhs_engStarvedSince", \
    "bmkhs_engClutchSlip", \
    "bmkhs_engSlipT", \
    "bmkhs_engSlipWait", \
    "bmkhs_engSlipDepth", \
    "bmkhs_shiftLocked", \
    "bmkhs_xmsnOutputRpm", \
    "bmkhs_xmsnDeltaRpm", \
    "bmkhs_reqEngTorque" \
]
