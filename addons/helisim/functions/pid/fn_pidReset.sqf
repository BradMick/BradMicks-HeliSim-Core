params ["_pid"];

//No previous error - the next run starts from its own, so engaging a loop is not a step
_pid deleteAt "prevError";
_pid set ["integral",  0];
_pid set ["derivFilt", 0];
