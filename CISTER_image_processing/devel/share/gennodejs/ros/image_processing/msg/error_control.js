// Auto-generated. Do not edit!

// (in-package image_processing.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class error_control {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.error_steer = null;
      this.control_error_steer = null;
      this.steer_integral = null;
      this.steer_deriv = null;
      this.pid_error_value = null;
      this.theta_error_value = null;
      this.dist_tv = null;
      this.status = null;
    }
    else {
      if (initObj.hasOwnProperty('error_steer')) {
        this.error_steer = initObj.error_steer
      }
      else {
        this.error_steer = 0.0;
      }
      if (initObj.hasOwnProperty('control_error_steer')) {
        this.control_error_steer = initObj.control_error_steer
      }
      else {
        this.control_error_steer = 0.0;
      }
      if (initObj.hasOwnProperty('steer_integral')) {
        this.steer_integral = initObj.steer_integral
      }
      else {
        this.steer_integral = 0.0;
      }
      if (initObj.hasOwnProperty('steer_deriv')) {
        this.steer_deriv = initObj.steer_deriv
      }
      else {
        this.steer_deriv = 0.0;
      }
      if (initObj.hasOwnProperty('pid_error_value')) {
        this.pid_error_value = initObj.pid_error_value
      }
      else {
        this.pid_error_value = 0.0;
      }
      if (initObj.hasOwnProperty('theta_error_value')) {
        this.theta_error_value = initObj.theta_error_value
      }
      else {
        this.theta_error_value = 0.0;
      }
      if (initObj.hasOwnProperty('dist_tv')) {
        this.dist_tv = initObj.dist_tv
      }
      else {
        this.dist_tv = 0.0;
      }
      if (initObj.hasOwnProperty('status')) {
        this.status = initObj.status
      }
      else {
        this.status = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type error_control
    // Serialize message field [error_steer]
    bufferOffset = _serializer.float32(obj.error_steer, buffer, bufferOffset);
    // Serialize message field [control_error_steer]
    bufferOffset = _serializer.float32(obj.control_error_steer, buffer, bufferOffset);
    // Serialize message field [steer_integral]
    bufferOffset = _serializer.float32(obj.steer_integral, buffer, bufferOffset);
    // Serialize message field [steer_deriv]
    bufferOffset = _serializer.float32(obj.steer_deriv, buffer, bufferOffset);
    // Serialize message field [pid_error_value]
    bufferOffset = _serializer.float32(obj.pid_error_value, buffer, bufferOffset);
    // Serialize message field [theta_error_value]
    bufferOffset = _serializer.float32(obj.theta_error_value, buffer, bufferOffset);
    // Serialize message field [dist_tv]
    bufferOffset = _serializer.float32(obj.dist_tv, buffer, bufferOffset);
    // Serialize message field [status]
    bufferOffset = _serializer.int32(obj.status, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type error_control
    let len;
    let data = new error_control(null);
    // Deserialize message field [error_steer]
    data.error_steer = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [control_error_steer]
    data.control_error_steer = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [steer_integral]
    data.steer_integral = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [steer_deriv]
    data.steer_deriv = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [pid_error_value]
    data.pid_error_value = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [theta_error_value]
    data.theta_error_value = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [dist_tv]
    data.dist_tv = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [status]
    data.status = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 32;
  }

  static datatype() {
    // Returns string type for a message object
    return 'image_processing/error_control';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1d6a944f0c56f70ef6f19b82ff1f68dd';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 error_steer
    float32 control_error_steer
    float32 steer_integral
    float32 steer_deriv
    float32 pid_error_value
    float32 theta_error_value
    float32 dist_tv
    int32 status
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new error_control(null);
    if (msg.error_steer !== undefined) {
      resolved.error_steer = msg.error_steer;
    }
    else {
      resolved.error_steer = 0.0
    }

    if (msg.control_error_steer !== undefined) {
      resolved.control_error_steer = msg.control_error_steer;
    }
    else {
      resolved.control_error_steer = 0.0
    }

    if (msg.steer_integral !== undefined) {
      resolved.steer_integral = msg.steer_integral;
    }
    else {
      resolved.steer_integral = 0.0
    }

    if (msg.steer_deriv !== undefined) {
      resolved.steer_deriv = msg.steer_deriv;
    }
    else {
      resolved.steer_deriv = 0.0
    }

    if (msg.pid_error_value !== undefined) {
      resolved.pid_error_value = msg.pid_error_value;
    }
    else {
      resolved.pid_error_value = 0.0
    }

    if (msg.theta_error_value !== undefined) {
      resolved.theta_error_value = msg.theta_error_value;
    }
    else {
      resolved.theta_error_value = 0.0
    }

    if (msg.dist_tv !== undefined) {
      resolved.dist_tv = msg.dist_tv;
    }
    else {
      resolved.dist_tv = 0.0
    }

    if (msg.status !== undefined) {
      resolved.status = msg.status;
    }
    else {
      resolved.status = 0
    }

    return resolved;
    }
};

module.exports = error_control;
