/* ----------------------------------------------------------------------------
Function: bmkhs_fnc_suspensionPlane

Description:
    The upward normal of the least-squares plane through a set of points.

Parameters:
    _points - Three or more positions [Array]

Returns:
    Unit normal, z positive [Array], or [] when the points are in a line

Author:
    BradMick
---------------------------------------------------------------------------- */
params ["_points"];

private _n   = count _points;
if (_n < 3) exitWith { [] };

//About the centroid, so the constant term drops out and world coordinates stay well conditioned
private _mid = [0.0, 0.0, 0.0];
{ _mid = _mid vectorAdd _x } forEach _points;
_mid = _mid vectorMultiply (1.0 / _n);

private _sxx = 0.0;
private _sxy = 0.0;
private _syy = 0.0;
private _sxz = 0.0;
private _syz = 0.0;
{
    (_x vectorDiff _mid) params ["_px", "_py", "_pz"];
    _sxx = _sxx + (_px * _px);
    _sxy = _sxy + (_px * _py);
    _syy = _syy + (_py * _py);
    _sxz = _sxz + (_px * _pz);
    _syz = _syz + (_py * _pz);
} forEach _points;

//z = a x + b y
private _det = (_sxx * _syy) - (_sxy * _sxy);
if (abs _det < 0.000001) exitWith { [] };
private _a = ((_syy * _sxz) - (_sxy * _syz)) / _det;
private _b = ((_sxx * _syz) - (_sxy * _sxz)) / _det;

vectorNormalized [-_a, -_b, 1.0]
