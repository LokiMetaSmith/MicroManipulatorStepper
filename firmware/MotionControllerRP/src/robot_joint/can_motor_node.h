// --------------------------------------------------------------------------------------
// Project: MicroManipulatorStepper
// License: MIT (see LICENSE file for full description)
//          All text in here must be included in any redistribution.
// Author:  M. S. (diffraction limited)
// --------------------------------------------------------------------------------------

#pragma once

#include "robot_joint_interface.h"

//*** CLASS *****************************************************************************

class CANMotorNode : public IRobotJoint {
  public:
    CANMotorNode(int node_id);
    ~CANMotorNode() override;

    void init(int joint_idx) override;
    bool calibrate(bool print_measurements) override;
    void update_target(float p, float v) override;
    float get_position() override;
    float get_velocity() override;

    void set_enabled(bool enabled) override;

    bool is_homed() const override;
    bool is_calibrated() const override;

    void read_can_messages();
    void send_can_messages();

  private:
    int node_id;
    int joint_idx = 0;

    bool homed = false;
    bool calibrated = true; // Assume remote node handles its own internal calibration
    bool enabled = false;

    float current_position = 0.0f;
    float current_velocity = 0.0f;

    float target_position = 0.0f;
    float target_velocity = 0.0f;
};
