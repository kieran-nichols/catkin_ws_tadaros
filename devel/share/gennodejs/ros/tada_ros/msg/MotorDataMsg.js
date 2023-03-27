// Auto-generated. Do not edit!

// (in-package tada_ros.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class MotorDataMsg {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.mode = null;
      this.duration = null;
      this.motor1_move = null;
      this.motor2_move = null;
      this.motor1_torque = null;
      this.motor2_torque = null;
      this.PF = null;
      this.EV = null;
      this.CPU0 = null;
      this.CPU1 = null;
      this.CPU2 = null;
      this.CPU3 = null;
      this.t = null;
    }
    else {
      if (initObj.hasOwnProperty('mode')) {
        this.mode = initObj.mode
      }
      else {
        this.mode = 0;
      }
      if (initObj.hasOwnProperty('duration')) {
        this.duration = initObj.duration
      }
      else {
        this.duration = 0;
      }
      if (initObj.hasOwnProperty('motor1_move')) {
        this.motor1_move = initObj.motor1_move
      }
      else {
        this.motor1_move = 0;
      }
      if (initObj.hasOwnProperty('motor2_move')) {
        this.motor2_move = initObj.motor2_move
      }
      else {
        this.motor2_move = 0;
      }
      if (initObj.hasOwnProperty('motor1_torque')) {
        this.motor1_torque = initObj.motor1_torque
      }
      else {
        this.motor1_torque = 0;
      }
      if (initObj.hasOwnProperty('motor2_torque')) {
        this.motor2_torque = initObj.motor2_torque
      }
      else {
        this.motor2_torque = 0;
      }
      if (initObj.hasOwnProperty('PF')) {
        this.PF = initObj.PF
      }
      else {
        this.PF = 0.0;
      }
      if (initObj.hasOwnProperty('EV')) {
        this.EV = initObj.EV
      }
      else {
        this.EV = 0.0;
      }
      if (initObj.hasOwnProperty('CPU0')) {
        this.CPU0 = initObj.CPU0
      }
      else {
        this.CPU0 = 0.0;
      }
      if (initObj.hasOwnProperty('CPU1')) {
        this.CPU1 = initObj.CPU1
      }
      else {
        this.CPU1 = 0.0;
      }
      if (initObj.hasOwnProperty('CPU2')) {
        this.CPU2 = initObj.CPU2
      }
      else {
        this.CPU2 = 0.0;
      }
      if (initObj.hasOwnProperty('CPU3')) {
        this.CPU3 = initObj.CPU3
      }
      else {
        this.CPU3 = 0.0;
      }
      if (initObj.hasOwnProperty('t')) {
        this.t = initObj.t
      }
      else {
        this.t = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MotorDataMsg
    // Serialize message field [mode]
    bufferOffset = _serializer.int32(obj.mode, buffer, bufferOffset);
    // Serialize message field [duration]
    bufferOffset = _serializer.int32(obj.duration, buffer, bufferOffset);
    // Serialize message field [motor1_move]
    bufferOffset = _serializer.int32(obj.motor1_move, buffer, bufferOffset);
    // Serialize message field [motor2_move]
    bufferOffset = _serializer.int32(obj.motor2_move, buffer, bufferOffset);
    // Serialize message field [motor1_torque]
    bufferOffset = _serializer.int32(obj.motor1_torque, buffer, bufferOffset);
    // Serialize message field [motor2_torque]
    bufferOffset = _serializer.int32(obj.motor2_torque, buffer, bufferOffset);
    // Serialize message field [PF]
    bufferOffset = _serializer.float32(obj.PF, buffer, bufferOffset);
    // Serialize message field [EV]
    bufferOffset = _serializer.float32(obj.EV, buffer, bufferOffset);
    // Serialize message field [CPU0]
    bufferOffset = _serializer.float32(obj.CPU0, buffer, bufferOffset);
    // Serialize message field [CPU1]
    bufferOffset = _serializer.float32(obj.CPU1, buffer, bufferOffset);
    // Serialize message field [CPU2]
    bufferOffset = _serializer.float32(obj.CPU2, buffer, bufferOffset);
    // Serialize message field [CPU3]
    bufferOffset = _serializer.float32(obj.CPU3, buffer, bufferOffset);
    // Serialize message field [t]
    bufferOffset = _serializer.float32(obj.t, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MotorDataMsg
    let len;
    let data = new MotorDataMsg(null);
    // Deserialize message field [mode]
    data.mode = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [duration]
    data.duration = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [motor1_move]
    data.motor1_move = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [motor2_move]
    data.motor2_move = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [motor1_torque]
    data.motor1_torque = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [motor2_torque]
    data.motor2_torque = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [PF]
    data.PF = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [EV]
    data.EV = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [CPU0]
    data.CPU0 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [CPU1]
    data.CPU1 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [CPU2]
    data.CPU2 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [CPU3]
    data.CPU3 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [t]
    data.t = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 52;
  }

  static datatype() {
    // Returns string type for a message object
    return 'tada_ros/MotorDataMsg';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '9af5263dec48e390e62e66f239203e3c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 mode
    int32 duration
    int32 motor1_move
    int32 motor2_move
    int32 motor1_torque
    int32 motor2_torque
    float32 PF
    float32 EV
    float32 CPU0
    float32 CPU1
    float32 CPU2
    float32 CPU3
    float32 t
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new MotorDataMsg(null);
    if (msg.mode !== undefined) {
      resolved.mode = msg.mode;
    }
    else {
      resolved.mode = 0
    }

    if (msg.duration !== undefined) {
      resolved.duration = msg.duration;
    }
    else {
      resolved.duration = 0
    }

    if (msg.motor1_move !== undefined) {
      resolved.motor1_move = msg.motor1_move;
    }
    else {
      resolved.motor1_move = 0
    }

    if (msg.motor2_move !== undefined) {
      resolved.motor2_move = msg.motor2_move;
    }
    else {
      resolved.motor2_move = 0
    }

    if (msg.motor1_torque !== undefined) {
      resolved.motor1_torque = msg.motor1_torque;
    }
    else {
      resolved.motor1_torque = 0
    }

    if (msg.motor2_torque !== undefined) {
      resolved.motor2_torque = msg.motor2_torque;
    }
    else {
      resolved.motor2_torque = 0
    }

    if (msg.PF !== undefined) {
      resolved.PF = msg.PF;
    }
    else {
      resolved.PF = 0.0
    }

    if (msg.EV !== undefined) {
      resolved.EV = msg.EV;
    }
    else {
      resolved.EV = 0.0
    }

    if (msg.CPU0 !== undefined) {
      resolved.CPU0 = msg.CPU0;
    }
    else {
      resolved.CPU0 = 0.0
    }

    if (msg.CPU1 !== undefined) {
      resolved.CPU1 = msg.CPU1;
    }
    else {
      resolved.CPU1 = 0.0
    }

    if (msg.CPU2 !== undefined) {
      resolved.CPU2 = msg.CPU2;
    }
    else {
      resolved.CPU2 = 0.0
    }

    if (msg.CPU3 !== undefined) {
      resolved.CPU3 = msg.CPU3;
    }
    else {
      resolved.CPU3 = 0.0
    }

    if (msg.t !== undefined) {
      resolved.t = msg.t;
    }
    else {
      resolved.t = 0.0
    }

    return resolved;
    }
};

module.exports = MotorDataMsg;
