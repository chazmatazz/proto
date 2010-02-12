/* User plug-in point for simulator customizations
Copyright (C) 2005-2008, Jonathan Bachrach, Jacob Beal, and contributors 
listed in the AUTHORS file in the MIT Proto distribution's top directory.

This file is part of MIT Proto, and is distributed under the terms of
the GNU General Public License, with a linking exception, as described
in the file LICENSE in the MIT Proto distribution's top directory. */

// This file is where declarations for simulator customizations
// can be put.  By default, it is empty.

#include "DeviceTimer.h"
#include "TimeModel.h"
#include "sim-hardware.h"

#ifndef __CUSTOMIZATIONS__
#define __CUSTOMIZATIONS__

class FixedTimer : public DeviceTimer {
  SECONDS dt, half_dt, internal_dt, internal_half_dt;
  flo ratio;
public:
  FixedTimer(flo dt, flo ratio);

  void next_transmit(SECONDS* d_true, SECONDS* d_internal);

  void next_compute(SECONDS* d_true, SECONDS* d_internal);

  DeviceTimer* clone_device() { return new FixedTimer(dt,internal_dt/dt); }
  void set_internal_dt(SECONDS dt);

};

class FixedIntervalTime : public TimeModel, public HardwarePatch {
  BOOL sync;
  flo dt; flo var;
  flo ratio; flo rvar;  // ratio is internal/true time
public:
  FixedIntervalTime(Args* args, SpatialComputer* p);

  DeviceTimer* next_timer(SECONDS* start_lag);

  SECONDS cycle_time() { return dt; }
  NUM_VAL set_dt (NUM_VAL dt);

};

#endif // __CUSTOMIZATIONS__
