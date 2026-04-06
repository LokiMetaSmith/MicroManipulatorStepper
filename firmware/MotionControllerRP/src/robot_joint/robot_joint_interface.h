// --------------------------------------------------------------------------------------
// Project: MicroManipulatorStepper
// License: MIT (see LICENSE file for full description)
//          All text in here must be included in any redistribution.
// Author:  M. S. (diffraction limited)
// --------------------------------------------------------------------------------------

#pragma once

#include <stdint.h>

//*** CLASS *****************************************************************************

class IRobotJoint {
  public:
    virtual ~IRobotJoint() = default;

    virtual void init(int joint_idx) = 0;
    virtual bool calibrate(bool print_measurements) = 0;
    virtual void update_target(float p, float v) = 0;
    virtual float get_position() = 0;
    virtual float get_velocity() = 0;

    virtual void set_enabled(bool enabled) = 0;

    virtual bool is_homed() const = 0;
    virtual bool is_calibrated() const = 0;
};
