#pragma once

// Placeholder task for a future custom mode; currently identical to Position mode.
#include "FlightTaskManualPosition.hpp"

// A pure copy of Position mode: zero behavior changes, just reachable
// under its own name/mode slot. This mirrors how FlightTaskAltitudeCruise
// subclasses FlightTaskManualAltitudeSmoothVel -- except here nothing is
// overridden yet. Add overrides here later once this confirmed builds and
// switches correctly.
class FlightTaskMyNewMode : public FlightTaskManualPosition
{
public:
	using FlightTaskManualPosition::FlightTaskManualPosition;
	virtual ~FlightTaskMyNewMode() = default;
};
