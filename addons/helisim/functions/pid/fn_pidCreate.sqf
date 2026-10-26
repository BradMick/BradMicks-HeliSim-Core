params ["_kp", "_ki", "_kd", "_limit"];

//_limit - most the integral can push, output units; ki_clamp is its error-seconds
createHashMapFromArray
    [ ["kp",       _kp]
    , ["ki",       _ki]
    , ["kd",       _kd]
    , ["ki_clamp", if (_ki != 0) then { _limit / _ki } else { 0 }]
    , ["integral",  0]
    ];
