/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_systemCircuitFeed

Description:
    Records what one component is putting onto one circuit, and recomputes the
    node from every contribution.

    A node holds each feeder's value separately rather than being accumulated
    and reset each frame. That is what lets a component sleep: one that did not
    run still counts, because its contribution is stored rather than rebuilt.

    Highest feeder wins, as it did before.

Parameters:
    _heli     - The helicopter [Object]
    _circuit  - Node being fed [String]
    _source   - What is feeding it, its published variable name [String]
    _value    - What it is contributing [Number]
    _producer - true for a producer or converter, false for storage. Storage
                asks whether anything ELSE feeds its output, so the two are
                counted apart [Bool]

Returns:
    Whether the NODE's value changed, which is what the walk propagates from
    [Bool]

Author:
    BradMick
---------------------------------------------------------------------------- */
params ["_heli", "_circuit", "_source", "_value", ["_producer", true]];

if (_circuit == "") exitWith {false};

private _feeds = _heli getVariable ["bmkhs_sysFeeds", createHashMap];
private _node  = _feeds getOrDefault [_circuit, createHashMap];

//Nothing to do if this feeder is already contributing exactly this.
if ((_node getOrDefault [_source, -1]) isEqualTo _value) exitWith {false};

_node set [_source, _value];
_feeds set [_circuit, _node];
_heli setVariable ["bmkhs_sysFeeds", _feeds];

//Recompute the node, and the producer-only total storage compares itself against.
private _total    = 0;
private _fromProd = 0;
//Keyed by circuit AND source: one component feeds several nodes, and several feed one
//node, so a flag per component alone gets overwritten by whichever wrote last.
private _prodKeys = _heli getVariable ["bmkhs_sysFeedIsProducer", createHashMap];
_prodKeys set [_circuit + "/" + _source, _producer];
_heli setVariable ["bmkhs_sysFeedIsProducer", _prodKeys];

{
    private _v = _node get _x;
    _total = _total max _v;
    if (_prodKeys getOrDefault [_circuit + "/" + _x, true]) then { _fromProd = _fromProd max _v };
} forEach (keys _node);

private _circuits = _heli getVariable ["bmkhs_sysCircuits", createHashMap];
private _was = _circuits getOrDefault [_circuit, 0];
_circuits set [_circuit, _total];
_heli setVariable ["bmkhs_sysCircuits", _circuits];
_heli setVariable ["bmkhs_sysProducerFeed_" + _circuit, _fromProd];

//One feeder changing does not necessarily move the node - highest wins, so a lesser
//feeder rising under the winner changes nothing anyone reads.
_total != _was
