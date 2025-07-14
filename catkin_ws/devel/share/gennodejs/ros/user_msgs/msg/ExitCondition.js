// Auto-generated. Do not edit!

// (in-package user_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class ExitCondition {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.exit_l = null;
      this.exit_a = null;
      this.arrive_cnt = null;
    }
    else {
      if (initObj.hasOwnProperty('exit_l')) {
        this.exit_l = initObj.exit_l
      }
      else {
        this.exit_l = 0.0;
      }
      if (initObj.hasOwnProperty('exit_a')) {
        this.exit_a = initObj.exit_a
      }
      else {
        this.exit_a = 0.0;
      }
      if (initObj.hasOwnProperty('arrive_cnt')) {
        this.arrive_cnt = initObj.arrive_cnt
      }
      else {
        this.arrive_cnt = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ExitCondition
    // Serialize message field [exit_l]
    bufferOffset = _serializer.float64(obj.exit_l, buffer, bufferOffset);
    // Serialize message field [exit_a]
    bufferOffset = _serializer.float64(obj.exit_a, buffer, bufferOffset);
    // Serialize message field [arrive_cnt]
    bufferOffset = _serializer.uint8(obj.arrive_cnt, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ExitCondition
    let len;
    let data = new ExitCondition(null);
    // Deserialize message field [exit_l]
    data.exit_l = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [exit_a]
    data.exit_a = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [arrive_cnt]
    data.arrive_cnt = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 17;
  }

  static datatype() {
    // Returns string type for a message object
    return 'user_msgs/ExitCondition';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '7aec7e7f502d48d3f927e4aa6e30673c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 exit_l  # m
    float64 exit_a  # °
    uint8 arrive_cnt
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ExitCondition(null);
    if (msg.exit_l !== undefined) {
      resolved.exit_l = msg.exit_l;
    }
    else {
      resolved.exit_l = 0.0
    }

    if (msg.exit_a !== undefined) {
      resolved.exit_a = msg.exit_a;
    }
    else {
      resolved.exit_a = 0.0
    }

    if (msg.arrive_cnt !== undefined) {
      resolved.arrive_cnt = msg.arrive_cnt;
    }
    else {
      resolved.arrive_cnt = 0
    }

    return resolved;
    }
};

module.exports = ExitCondition;
