params ["_kp", "_ki", "_kd", "_ki_clamp"];

createHashMapFromArray
    [ ["kp",       _kp]
    , ["ki",       _ki]
    , ["kd",       _kd]
    , ["ki_clamp", _ki_clamp]
    , ["integral",  0]
    ];
