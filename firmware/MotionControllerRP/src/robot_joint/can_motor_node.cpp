// --------------------------------------------------------------------------------------
// Project: MicroManipulatorStepper
// License: MIT (see LICENSE file for full description)
//          All text in here must be included in any redistribution.
// Author:  M. S. (diffraction limited)
// --------------------------------------------------------------------------------------

#include "can_motor_node.h"

CANMotorNode::CANMotorNode(int node_id) : node_id(node_id) {
}

CANMotorNode::~CANMotorNode() {
}

void CANMotorNode::init(int joint_idx) {
    this->joint_idx = joint_idx;
    // Initialization of CAN node logic (waiting for heartbeat, etc.)
}

bool CANMotorNode::calibrate(bool print_measurements) {
    // The smart node handles its own calibration.
    // Send calibration command if necessary over CAN.
    calibrated = true;
    return true;
}

void CANMotorNode::update_target(float p, float v) {
    target_position = p;
    target_velocity = v;
}

float CANMotorNode::get_position() {
    return current_position;
}

float CANMotorNode::get_velocity() {
    return current_velocity;
}

void CANMotorNode::set_enabled(bool enabled) {
    this->enabled = enabled;
    // Send enable/disable command over CAN
}

bool CANMotorNode::is_homed() const {
    return homed;
}

bool CANMotorNode::is_calibrated() const {
    return calibrated;
}

void CANMotorNode::read_can_messages() {
    // Poll the CAN bus or read from interrupt buffer
    // Update current_position and current_velocity
    // Check homing status
}

void CANMotorNode::send_can_messages() {
    // Prepare standard CAN packet with node_id
    // Payload contains float target_position
    // can.write(CanMsg(CanStandardId(node_id), 4, (uint8_t*)&target_position));
}
