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

class coords {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.X2 = null;
      this.X1 = null;
      this.dif_X = null;
      this.Y2 = null;
      this.Y1 = null;
      this.dif_Y = null;
      this.slope = null;
      this.length = null;
      this.intercept = null;
    }
    else {
      if (initObj.hasOwnProperty('X2')) {
        this.X2 = initObj.X2
      }
      else {
        this.X2 = 0.0;
      }
      if (initObj.hasOwnProperty('X1')) {
        this.X1 = initObj.X1
      }
      else {
        this.X1 = 0.0;
      }
      if (initObj.hasOwnProperty('dif_X')) {
        this.dif_X = initObj.dif_X
      }
      else {
        this.dif_X = 0.0;
      }
      if (initObj.hasOwnProperty('Y2')) {
        this.Y2 = initObj.Y2
      }
      else {
        this.Y2 = 0.0;
      }
      if (initObj.hasOwnProperty('Y1')) {
        this.Y1 = initObj.Y1
      }
      else {
        this.Y1 = 0.0;
      }
      if (initObj.hasOwnProperty('dif_Y')) {
        this.dif_Y = initObj.dif_Y
      }
      else {
        this.dif_Y = 0.0;
      }
      if (initObj.hasOwnProperty('slope')) {
        this.slope = initObj.slope
      }
      else {
        this.slope = 0.0;
      }
      if (initObj.hasOwnProperty('length')) {
        this.length = initObj.length
      }
      else {
        this.length = 0.0;
      }
      if (initObj.hasOwnProperty('intercept')) {
        this.intercept = initObj.intercept
      }
      else {
        this.intercept = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type coords
    // Serialize message field [X2]
    bufferOffset = _serializer.float32(obj.X2, buffer, bufferOffset);
    // Serialize message field [X1]
    bufferOffset = _serializer.float32(obj.X1, buffer, bufferOffset);
    // Serialize message field [dif_X]
    bufferOffset = _serializer.float32(obj.dif_X, buffer, bufferOffset);
    // Serialize message field [Y2]
    bufferOffset = _serializer.float32(obj.Y2, buffer, bufferOffset);
    // Serialize message field [Y1]
    bufferOffset = _serializer.float32(obj.Y1, buffer, bufferOffset);
    // Serialize message field [dif_Y]
    bufferOffset = _serializer.float32(obj.dif_Y, buffer, bufferOffset);
    // Serialize message field [slope]
    bufferOffset = _serializer.float32(obj.slope, buffer, bufferOffset);
    // Serialize message field [length]
    bufferOffset = _serializer.float32(obj.length, buffer, bufferOffset);
    // Serialize message field [intercept]
    bufferOffset = _serializer.float32(obj.intercept, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type coords
    let len;
    let data = new coords(null);
    // Deserialize message field [X2]
    data.X2 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [X1]
    data.X1 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [dif_X]
    data.dif_X = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [Y2]
    data.Y2 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [Y1]
    data.Y1 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [dif_Y]
    data.dif_Y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [slope]
    data.slope = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [length]
    data.length = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [intercept]
    data.intercept = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 36;
  }

  static datatype() {
    // Returns string type for a message object
    return 'image_processing/coords';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '07271e516a6d3af46ff39e3801eabeb2';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 X2
    float32 X1
    float32 dif_X
    float32 Y2
    float32 Y1
    float32 dif_Y
    float32 slope
    float32 length
    float32 intercept
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new coords(null);
    if (msg.X2 !== undefined) {
      resolved.X2 = msg.X2;
    }
    else {
      resolved.X2 = 0.0
    }

    if (msg.X1 !== undefined) {
      resolved.X1 = msg.X1;
    }
    else {
      resolved.X1 = 0.0
    }

    if (msg.dif_X !== undefined) {
      resolved.dif_X = msg.dif_X;
    }
    else {
      resolved.dif_X = 0.0
    }

    if (msg.Y2 !== undefined) {
      resolved.Y2 = msg.Y2;
    }
    else {
      resolved.Y2 = 0.0
    }

    if (msg.Y1 !== undefined) {
      resolved.Y1 = msg.Y1;
    }
    else {
      resolved.Y1 = 0.0
    }

    if (msg.dif_Y !== undefined) {
      resolved.dif_Y = msg.dif_Y;
    }
    else {
      resolved.dif_Y = 0.0
    }

    if (msg.slope !== undefined) {
      resolved.slope = msg.slope;
    }
    else {
      resolved.slope = 0.0
    }

    if (msg.length !== undefined) {
      resolved.length = msg.length;
    }
    else {
      resolved.length = 0.0
    }

    if (msg.intercept !== undefined) {
      resolved.intercept = msg.intercept;
    }
    else {
      resolved.intercept = 0.0
    }

    return resolved;
    }
};

module.exports = coords;
