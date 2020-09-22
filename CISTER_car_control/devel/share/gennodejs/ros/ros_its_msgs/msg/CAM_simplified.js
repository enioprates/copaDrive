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

class CAM_simplified {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.car_name = null;
      this.Station_ID = null;
      this.latitude = null;
      this.longitude = null;
      this.altitude_altitudeValue = null;
      this.heading_headingValue = null;
      this.speed_speedValue = null;
      this.driveDirection = null;
      this.steeringWheelAngle_steeringWheelAngleValue = null;
      this.gasPedalPercent_Value = null;
      this.brakePedalPercent_Value = null;
    }
    else {
      if (initObj.hasOwnProperty('car_name')) {
        this.car_name = initObj.car_name
      }
      else {
        this.car_name = '';
      }
      if (initObj.hasOwnProperty('Station_ID')) {
        this.Station_ID = initObj.Station_ID
      }
      else {
        this.Station_ID = '';
      }
      if (initObj.hasOwnProperty('latitude')) {
        this.latitude = initObj.latitude
      }
      else {
        this.latitude = 0.0;
      }
      if (initObj.hasOwnProperty('longitude')) {
        this.longitude = initObj.longitude
      }
      else {
        this.longitude = 0.0;
      }
      if (initObj.hasOwnProperty('altitude_altitudeValue')) {
        this.altitude_altitudeValue = initObj.altitude_altitudeValue
      }
      else {
        this.altitude_altitudeValue = 0.0;
      }
      if (initObj.hasOwnProperty('heading_headingValue')) {
        this.heading_headingValue = initObj.heading_headingValue
      }
      else {
        this.heading_headingValue = 0.0;
      }
      if (initObj.hasOwnProperty('speed_speedValue')) {
        this.speed_speedValue = initObj.speed_speedValue
      }
      else {
        this.speed_speedValue = 0.0;
      }
      if (initObj.hasOwnProperty('driveDirection')) {
        this.driveDirection = initObj.driveDirection
      }
      else {
        this.driveDirection = 0;
      }
      if (initObj.hasOwnProperty('steeringWheelAngle_steeringWheelAngleValue')) {
        this.steeringWheelAngle_steeringWheelAngleValue = initObj.steeringWheelAngle_steeringWheelAngleValue
      }
      else {
        this.steeringWheelAngle_steeringWheelAngleValue = 0.0;
      }
      if (initObj.hasOwnProperty('gasPedalPercent_Value')) {
        this.gasPedalPercent_Value = initObj.gasPedalPercent_Value
      }
      else {
        this.gasPedalPercent_Value = 0.0;
      }
      if (initObj.hasOwnProperty('brakePedalPercent_Value')) {
        this.brakePedalPercent_Value = initObj.brakePedalPercent_Value
      }
      else {
        this.brakePedalPercent_Value = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CAM_simplified
    // Serialize message field [car_name]
    bufferOffset = _serializer.string(obj.car_name, buffer, bufferOffset);
    // Serialize message field [Station_ID]
    bufferOffset = _serializer.string(obj.Station_ID, buffer, bufferOffset);
    // Serialize message field [latitude]
    bufferOffset = _serializer.float32(obj.latitude, buffer, bufferOffset);
    // Serialize message field [longitude]
    bufferOffset = _serializer.float64(obj.longitude, buffer, bufferOffset);
    // Serialize message field [altitude_altitudeValue]
    bufferOffset = _serializer.float32(obj.altitude_altitudeValue, buffer, bufferOffset);
    // Serialize message field [heading_headingValue]
    bufferOffset = _serializer.float32(obj.heading_headingValue, buffer, bufferOffset);
    // Serialize message field [speed_speedValue]
    bufferOffset = _serializer.float32(obj.speed_speedValue, buffer, bufferOffset);
    // Serialize message field [driveDirection]
    bufferOffset = _serializer.int8(obj.driveDirection, buffer, bufferOffset);
    // Serialize message field [steeringWheelAngle_steeringWheelAngleValue]
    bufferOffset = _serializer.float32(obj.steeringWheelAngle_steeringWheelAngleValue, buffer, bufferOffset);
    // Serialize message field [gasPedalPercent_Value]
    bufferOffset = _serializer.float32(obj.gasPedalPercent_Value, buffer, bufferOffset);
    // Serialize message field [brakePedalPercent_Value]
    bufferOffset = _serializer.float32(obj.brakePedalPercent_Value, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CAM_simplified
    let len;
    let data = new CAM_simplified(null);
    // Deserialize message field [car_name]
    data.car_name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [Station_ID]
    data.Station_ID = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [latitude]
    data.latitude = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [longitude]
    data.longitude = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [altitude_altitudeValue]
    data.altitude_altitudeValue = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [heading_headingValue]
    data.heading_headingValue = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [speed_speedValue]
    data.speed_speedValue = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [driveDirection]
    data.driveDirection = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [steeringWheelAngle_steeringWheelAngleValue]
    data.steeringWheelAngle_steeringWheelAngleValue = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [gasPedalPercent_Value]
    data.gasPedalPercent_Value = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [brakePedalPercent_Value]
    data.brakePedalPercent_Value = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.car_name.length;
    length += object.Station_ID.length;
    return length + 45;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ros_its_msgs/CAM_simplified';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5ad8a1ab31db5a9f774bad039fc1ce70';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # IDs
    
    string car_name
    string Station_ID
    
    # Reference Position
    #int32 latitude
    #int64 longitude
    #int32 altitude_altitudeValue
    
    float32 latitude
    float64 longitude
    float32 altitude_altitudeValue
    
    # BasicVehicleContainerHighFrequency (Simplified)
    
    #uint16 heading_headingValue
    #uint16 speed_speedValue
    float32 heading_headingValue
    float32 speed_speedValue
    
    int8 driveDirection
    
    #int16 steeringWheelAngle_steeringWheelAngleValue
    float32 steeringWheelAngle_steeringWheelAngleValue
    
    float32 gasPedalPercent_Value
    float32 brakePedalPercent_Value
    
    #float32 Ref_distance
    #float32 leader_distance
    
    
    
    
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new CAM_simplified(null);
    if (msg.car_name !== undefined) {
      resolved.car_name = msg.car_name;
    }
    else {
      resolved.car_name = ''
    }

    if (msg.Station_ID !== undefined) {
      resolved.Station_ID = msg.Station_ID;
    }
    else {
      resolved.Station_ID = ''
    }

    if (msg.latitude !== undefined) {
      resolved.latitude = msg.latitude;
    }
    else {
      resolved.latitude = 0.0
    }

    if (msg.longitude !== undefined) {
      resolved.longitude = msg.longitude;
    }
    else {
      resolved.longitude = 0.0
    }

    if (msg.altitude_altitudeValue !== undefined) {
      resolved.altitude_altitudeValue = msg.altitude_altitudeValue;
    }
    else {
      resolved.altitude_altitudeValue = 0.0
    }

    if (msg.heading_headingValue !== undefined) {
      resolved.heading_headingValue = msg.heading_headingValue;
    }
    else {
      resolved.heading_headingValue = 0.0
    }

    if (msg.speed_speedValue !== undefined) {
      resolved.speed_speedValue = msg.speed_speedValue;
    }
    else {
      resolved.speed_speedValue = 0.0
    }

    if (msg.driveDirection !== undefined) {
      resolved.driveDirection = msg.driveDirection;
    }
    else {
      resolved.driveDirection = 0
    }

    if (msg.steeringWheelAngle_steeringWheelAngleValue !== undefined) {
      resolved.steeringWheelAngle_steeringWheelAngleValue = msg.steeringWheelAngle_steeringWheelAngleValue;
    }
    else {
      resolved.steeringWheelAngle_steeringWheelAngleValue = 0.0
    }

    if (msg.gasPedalPercent_Value !== undefined) {
      resolved.gasPedalPercent_Value = msg.gasPedalPercent_Value;
    }
    else {
      resolved.gasPedalPercent_Value = 0.0
    }

    if (msg.brakePedalPercent_Value !== undefined) {
      resolved.brakePedalPercent_Value = msg.brakePedalPercent_Value;
    }
    else {
      resolved.brakePedalPercent_Value = 0.0
    }

    return resolved;
    }
};

module.exports = CAM_simplified;
