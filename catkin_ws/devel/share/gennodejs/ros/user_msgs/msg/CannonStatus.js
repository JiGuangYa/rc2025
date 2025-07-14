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

class CannonStatus {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.bullet_num = null;
      this.capacity = null;
      this.reload_time = null;
      this.rdy2fire = null;
    }
    else {
      if (initObj.hasOwnProperty('bullet_num')) {
        this.bullet_num = initObj.bullet_num
      }
      else {
        this.bullet_num = 0;
      }
      if (initObj.hasOwnProperty('capacity')) {
        this.capacity = initObj.capacity
      }
      else {
        this.capacity = 0;
      }
      if (initObj.hasOwnProperty('reload_time')) {
        this.reload_time = initObj.reload_time
      }
      else {
        this.reload_time = 0.0;
      }
      if (initObj.hasOwnProperty('rdy2fire')) {
        this.rdy2fire = initObj.rdy2fire
      }
      else {
        this.rdy2fire = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CannonStatus
    // Serialize message field [bullet_num]
    bufferOffset = _serializer.uint8(obj.bullet_num, buffer, bufferOffset);
    // Serialize message field [capacity]
    bufferOffset = _serializer.uint8(obj.capacity, buffer, bufferOffset);
    // Serialize message field [reload_time]
    bufferOffset = _serializer.float32(obj.reload_time, buffer, bufferOffset);
    // Serialize message field [rdy2fire]
    bufferOffset = _serializer.bool(obj.rdy2fire, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CannonStatus
    let len;
    let data = new CannonStatus(null);
    // Deserialize message field [bullet_num]
    data.bullet_num = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [capacity]
    data.capacity = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [reload_time]
    data.reload_time = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [rdy2fire]
    data.rdy2fire = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 7;
  }

  static datatype() {
    // Returns string type for a message object
    return 'user_msgs/CannonStatus';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e31433468f551973da97c9147997bd00';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint8 bullet_num
    
    uint8 capacity
    float32 reload_time
    
    bool rdy2fire
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new CannonStatus(null);
    if (msg.bullet_num !== undefined) {
      resolved.bullet_num = msg.bullet_num;
    }
    else {
      resolved.bullet_num = 0
    }

    if (msg.capacity !== undefined) {
      resolved.capacity = msg.capacity;
    }
    else {
      resolved.capacity = 0
    }

    if (msg.reload_time !== undefined) {
      resolved.reload_time = msg.reload_time;
    }
    else {
      resolved.reload_time = 0.0
    }

    if (msg.rdy2fire !== undefined) {
      resolved.rdy2fire = msg.rdy2fire;
    }
    else {
      resolved.rdy2fire = false
    }

    return resolved;
    }
};

module.exports = CannonStatus;
