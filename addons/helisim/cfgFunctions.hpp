// recompile = 1 UNCONDITIONALLY - this is what makes -filePatching work.
//
// This was previously gated behind `#ifdef __A3_DEBUG__`, which is DEAD CODE under HEMTT:
// __A3_DEBUG__ is one of HEMTT's "runtime macros", hardcoded to the value 0 in a lookup
// the #ifdef existence check never consults (libs/preprocessor/src/defines.rs). It cannot
// be defined - there is no project.toml key, no CLI flag, and no per-command scoping - so
// the #else branch always won and every function was built with recompile = 0.
//
// With recompile = 0 the engine compiles each function once and caches it forever, so a
// patched .sqf on disk is never re-read: file patching appears completely broken even with
// -filePatching enabled, correct addon junctions, and bytecode stripped from the PBO. There
// is no error; edits are simply ignored.
//
// Cost of leaving this on for release: effectively nothing. Without -filePatching there is
// no loose file to find, so the engine compiles the PBO's copy once at mission start exactly
// as it did before. This mirrors how CBA/ACE handle their dev-recompile switch (a runtime
// gate rather than a rapify-time define), adapted to a CfgFunctions-based project.
//
// NOTE: file patching only overrides paths that already exist in the built PBO - adding a
// BRAND NEW .sqf still requires a rebuild.
#define R recompile = 1

class CfgFunctions
{
    class bmkhs_helisim_project
    {
        tag = "bmkhs";
        class actuator {
            file = "\bmkhs_helisim\functions\actuator";
            class actuator {R;};
            class actuatorGetLagCoefA {R;};
            class actuatorGetLagCoefB {R;};
            class actuatorLag {R;};
            class actuatorVariables {R;};
        };
        class wing {
            file = "\bmkhs_helisim\functions\wing";
            class wing {R;};
            class wingUpdate {R;};
            class wingVariables {R;};
        };
        class airfoil {
            file = "\bmkhs_helisim\functions\airfoil";
            class airfoilGet {R;};
            class airfoilVariables {R;};
        };
        class core {
            file = "\bmkhs_helisim\functions\core";
            class coreConfig {R;};
            class coreInit {R;};
            class coreUpdate  {R;};
            class coreUpdateFlightModel {R;};
        };
        class ctrlVis {
            file = "\bmkhs_helisim\functions\ctrlVis";
            class ctrlVisToggle {R;};
            class ctrlVisUpdate {R;};
        };
        class damage {
            file = "\bmkhs_helisim\functions\damage";
            class damageApply {R;};
            class damageCount {R;};
            class damageGet {R;};
            class damageSet {R;};
            class damageVariables {R;};
        };
        class event {
            file = "\bmkhs_helisim\functions\event";
            class eventGetIn {R;};
        };
        class engine {
            file = "\bmkhs_helisim\functions\engine";
            class engine  {R;};
            class engine2 {R;};
            class engineBET {R;};
            class engineController {R;};
            class engineVariables {R;};
        };
        class environment {
            file = "\bmkhs_helisim\functions\environment";
            class environment {R;};
        };
        class fmc  {
            file = "\bmkhs_helisim\functions\fmc";
            class fmc {R;};
            class fmcAttitudeHold {R;};
            class fmcAltitudeHold {R;};
            class fmcAltitudeHoldEnable {R;};
            class fmcAttitudeHoldEnable {R;};
            class fmcControlMixing {R;};
            class fmcForceTrimHold {R;};
            class fmcForceTrimRelease {R;};
            class fmcForceTrimReset {R;};
            class fmcForceTrimSet {R;};
            class fmcSetChannel {R;};
            class fmcHeadingHold {R;};
            class fmcHoldModesDisable {R;};
            class fmcSAS {R;};
            class fmcVariables {R;};
        };
        class prestonAi {
            file = "\bmkhs_helisim\functions\prestonAi";
            class preston {R;};
            class prestonPedal {R;};
            class prestonPilot {R;};
            class prestonVariables {R;};
        };
        class fuel {
            file = "\bmkhs_helisim\functions\fuel";
            class fuelDraw {R;};
            class fuelTankVarName {R;};
            class fuelLeak {R;};
            class fuelMgmtUpdate {R;};
            class fuelSet {R;};
            class fuelTransfer {R;};
            class fuelTransferAux {R;};
            class fuelUpdate {R;};
            class fuelVariables {R;};
        };
        class fuselage {
            file = "\bmkhs_helisim\functions\fuselage";
            class fuselageUpdate {R;};
            class fuselageFront {R;};
            class fuselageSide {R;};
            class fuselageTop {R;};
            class fuselageVariables {R;};
        };
        class controls {
            file = "\bmkhs_helisim\functions\controls";
            class control {R;};
            class controlPublish {R;};
            class controlSet {R;};
            class controlsRelease {R;};
            class controlsUpdate {R;};
            class controlsVariables {R;};
        };
        class input {
            file = "\bmkhs_helisim\functions\input";
            class inputAnalogHandler {R;};
            class inputCenterTrimMode {R;};
            class inputControlHandle {R;};
            class inputUpdate {R;};
            class inputGetInterp {R;};
            class inputVariables {R;};
            class inputNonAnalogHandler {R;};
            class stickyInterrupt {R;};
        };
        class mass {
            file = "\bmkhs_helisim\functions\mass";
            class massUpdate {R;};
            class massVariables {R;};
            class massUpdateStation {R;};
        };
        class math {
            file = "\bmkhs_helisim\functions\math";
            class mathGetArea {R;};
            class mathIsINF {R;};
            class mathIsNAN {R;};
            class mathLinearInterp {R;};
            class mathLinearInterpFromCenter {R;};
            class mathRotateVector {R;};
            class mathVectorRotate {R;};
            class mathVectorRotateAroundAxis {R;};
        };
        class pid {
            file = "\bmkhs_helisim\functions\pid";
            class pidCreate {R;};
            class pidReset {R;};
            class pidRun {R;};
        };
        class util {
            file = "\bmkhs_helisim\functions\util";
            class utilSmoothAverage {R;};
            class utilNotify {R;};
            class utilSetArrayVariable {R;};
            class utilSetMultiArrayVariable {R;};
            class utilUpdateNetworkGlobal {R;};
        };
        class debug {
            file = "\bmkhs_helisim\functions\debug";
            class debugDrawCircle {R;};
            class debugDrawLine {R;};
        };
        class mathQuaternion {
            file = "\bmkhs_helisim\functions\math\quaternion";
            class quaternion {R;};
            class quaternionConjugate {R;};
            class quaternionFromVec3 {R;};
            class quaternionMultiply {R;};
            class quaternionNormalize {R;};
        };
        class mathSmoothAverage {
            file = "\bmkhs_helisim\functions\math\smoothAverage";
            class smoothAverageAdd {R;};
            class smoothAverageGet {R;};
            class smoothAverageInit {R;};
        };
        class performance {
            file = "\bmkhs_helisim\functions\performance";
            class perfData {R;};
            class perfVariables {R;};
        };
        class rotor {
            file = "\bmkhs_helisim\functions\rotor";
            class rotor {R;};
            class rotorBlade {R;};
            class rotorControl {R;};
            class rotorFlapDynamics {R;};
            class rotorUpdate {R;};
            class rotorVariables {R;};
        };
        class simpleRotor {
            file = "\bmkhs_helisim\functions\simpleRotor";
            class simpleRotorMain {R;};
            class simpleRotorTail {R;};
            class simpleRotorVariables {R;};
        };
        class state {
            file = "\bmkhs_helisim\functions\state";
            class stateAeroValues {R;};
            class stateAccelerations {R;};
            class stateVariables {R;};
            class stateDeltaTime {R;};
            class stateAltitude {R;};
            class stateRtrRPM {R;};
            class stateVelocities {R;};
            class stateOnGround {R;};
        };
        class systems {
            file = "\bmkhs_helisim\functions\systems";
            class systemCircuit {R;};
            class systemCircuitFeed {R;};
            class systemCircuitState {R;};
            class systemConsumer {R;};
            class systemConverter {R;};
            class systemProducer {R;};
            class systemStorage {R;};
            class systemTorque {R;};
            class systemTorqueJitter {R;};
            class systemsComponents {R;};
            class systemsDebug {R;};
            class systemsSolve {R;};
            class systemsUpdate {R;};
            class systemsVariables {R;};
        };
        class systemsApu {
            file = "\bmkhs_helisim\functions\systems\apu";
            class apu {R;};
        };
        class systemsRepair {
            file = "\bmkhs_helisim\functions\systems\repair";
            class repair {R;};
        };
        class transmission {
            file = "\bmkhs_helisim\functions\transmission";
            class transmissionUpdate {R;};
            class transmissionVariables {R;};
        };
        class utility {
            file = "\bmkhs_helisim\functions";
        };
    };
};
