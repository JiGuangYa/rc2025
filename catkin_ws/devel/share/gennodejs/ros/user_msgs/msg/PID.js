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

class PID {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.p = null;
      this.i = null;
      this.d = null;
      this.i_activate = null;
      this.out_thresh = null;
    }
    else {
      if (initObj.hasOwnProperty('p')) {
        this.p = initObj.p
      }
      else {
        this.p = 0.0;
      }
      if (initObj.hasOwnProperty('i')) {
        this.i = initObj.i
      }
      else {
        this.i = 0.0;
      }
      if (initObj.hasOwnProperty('d')) {
        this.d = initObj.d
      }
      else {
        this.d = 0.0;
      }
      if (initObj.hasOwnProperty('i_activate')) {
        this.i_activate = initObj.i_activate
      }
      else {
        this.i_activate = 0.0;
      }
      if (initObj.hasOwnProperty('out_thresh')) {
        this.out_thresh = initObj.out_thresh
      }
      else {
        this.out_thresh = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PID
    // Serialize message field [p]
    bufferOffset = _serializer.float32(obj.p, buffer, bufferOffset);
    // Serialize message field [i]
    bufferOffset = _serializer.float32(obj.i, buffer, bufferOffset);
    // Serialize message field [d]
    bufferOffset = _serializer.float32(obj.d, buffer, bufferOffset);
    // Serialize message field [i_activate]
    bufferOffset = _serializer.float32(obj.i_activate, buffer, bufferOffset);
    // Serialize message field [out_thresh]
    bufferOffset = _serializer.float32(obj.out_thresh, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PID
    let len;
    let data = new PID(null);
    // Deserialize message field [p]
    data.p = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [i]
    data.i = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [d]
    data.d = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [i_activate]
    data.i_activate = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [out_thresh]
    data.out_thresh = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 20;
  }

  static datatype() {
    // Returns string type for a message object
    return 'user_msgs/PID';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a7a91d2c8b17a0071e3c8256ef5a2554';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 p
    float32 i
    float32 d
    float32 i_activate
    float32 out_thresh
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new PID(null);
    if (msg.p !== undefined) {
      resolved.p = msg.p;
    }
    else {
      resolved.p = 0.0
    }

    if (msg.i !== undefined) {
      resolved.i = msg.i;
    }
    else {
      resolved.i = 0.0
    }

    if (msg.d !== undefined) {
      resolved.d = msg.d;
    }
    else {
      resolved.d = 0.0
    }

    if (msg.i_activate !== undefined) {
      resolved.i_activate = msg.i_activate;
    }
    else {
      resolved.i_activate = 0.0
    }

    if (msg.out_thresh !== undefined) {
      resolved.out_thresh = msg.out_thresh;
    }
    else {
      resolved.out_thresh = 0.0
    }

    return resolved;
    }
};

module.exports = PID;
