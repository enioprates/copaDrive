// Auto-generated. Do not edit!

// (in-package ros_its_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class CLWarning {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.car_name = null;
      this.CLWType = null;
    }
    else {
      if (initObj.hasOwnProperty('car_name')) {
        this.car_name = initObj.car_name
      }
      else {
        this.car_name = '';
      }
      if (initObj.hasOwnProperty('CLWType')) {
        this.CLWType = initObj.CLWType
      }
      else {
        this.CLWType = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CLWarning
    // Serialize message field [car_name]
    bufferOffset = _serializer.string(obj.car_name, buffer, bufferOffset);
    // Serialize message field [CLWType]
    bufferOffset = _serializer.string(obj.CLWType, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CLWarning
    let len;
    let data = new CLWarning(null);
    // Deserialize message field [car_name]
    data.car_name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [CLWType]
    data.CLWType = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.car_name.length;
    length += object.CLWType.length;
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ros_its_msgs/CLWarning';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a3e52a7449922f3c43207fb5cac4c802';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string car_name
    
    string CLWType
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new CLWarning(null);
    if (msg.car_name !== undefined) {
      resolved.car_name = msg.car_name;
    }
    else {
      resolved.car_name = ''
    }

    if (msg.CLWType !== undefined) {
      resolved.CLWType = msg.CLWType;
    }
    else {
      resolved.CLWType = ''
    }

    return resolved;
    }
};

module.exports = CLWarning;
