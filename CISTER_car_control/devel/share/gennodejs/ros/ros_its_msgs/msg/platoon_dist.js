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

class platoon_dist {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.car_name = null;
      this.Ref_distance = null;
      this.leader_distance = null;
    }
    else {
      if (initObj.hasOwnProperty('car_name')) {
        this.car_name = initObj.car_name
      }
      else {
        this.car_name = '';
      }
      if (initObj.hasOwnProperty('Ref_distance')) {
        this.Ref_distance = initObj.Ref_distance
      }
      else {
        this.Ref_distance = 0.0;
      }
      if (initObj.hasOwnProperty('leader_distance')) {
        this.leader_distance = initObj.leader_distance
      }
      else {
        this.leader_distance = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type platoon_dist
    // Serialize message field [car_name]
    bufferOffset = _serializer.string(obj.car_name, buffer, bufferOffset);
    // Serialize message field [Ref_distance]
    bufferOffset = _serializer.float32(obj.Ref_distance, buffer, bufferOffset);
    // Serialize message field [leader_distance]
    bufferOffset = _serializer.float32(obj.leader_distance, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type platoon_dist
    let len;
    let data = new platoon_dist(null);
    // Deserialize message field [car_name]
    data.car_name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [Ref_distance]
    data.Ref_distance = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [leader_distance]
    data.leader_distance = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.car_name.length;
    return length + 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ros_its_msgs/platoon_dist';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '865334224e65761f8e48bcb6f774f442';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string car_name
    
    float32 Ref_distance
    float32 leader_distance
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new platoon_dist(null);
    if (msg.car_name !== undefined) {
      resolved.car_name = msg.car_name;
    }
    else {
      resolved.car_name = ''
    }

    if (msg.Ref_distance !== undefined) {
      resolved.Ref_distance = msg.Ref_distance;
    }
    else {
      resolved.Ref_distance = 0.0
    }

    if (msg.leader_distance !== undefined) {
      resolved.leader_distance = msg.leader_distance;
    }
    else {
      resolved.leader_distance = 0.0
    }

    return resolved;
    }
};

module.exports = platoon_dist;
