// Auto-generated. Do not edit!

// (in-package user_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let ObjDet = require('./ObjDet.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class ObjDets {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.fps = null;
      this.num = null;
      this.dets = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('fps')) {
        this.fps = initObj.fps
      }
      else {
        this.fps = 0;
      }
      if (initObj.hasOwnProperty('num')) {
        this.num = initObj.num
      }
      else {
        this.num = 0;
      }
      if (initObj.hasOwnProperty('dets')) {
        this.dets = initObj.dets
      }
      else {
        this.dets = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ObjDets
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [fps]
    bufferOffset = _serializer.int16(obj.fps, buffer, bufferOffset);
    // Serialize message field [num]
    bufferOffset = _serializer.int16(obj.num, buffer, bufferOffset);
    // Serialize message field [dets]
    // Serialize the length for message field [dets]
    bufferOffset = _serializer.uint32(obj.dets.length, buffer, bufferOffset);
    obj.dets.forEach((val) => {
      bufferOffset = ObjDet.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ObjDets
    let len;
    let data = new ObjDets(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [fps]
    data.fps = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [num]
    data.num = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [dets]
    // Deserialize array length for message field [dets]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.dets = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.dets[i] = ObjDet.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    object.dets.forEach((val) => {
      length += ObjDet.getMessageSize(val);
    });
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'user_msgs/ObjDets';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1c6e1901031d8b3566cf3e9e9d7037d6';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    
    int16 fps
    
    int16 num
    
    ObjDet[] dets
    
    
    
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    ================================================================================
    MSG: user_msgs/ObjDet
    uint16 index
    
    uint8 class_id
    
    string label
    
    float64 score
    
    ObjBbox bbox
    
    geometry_msgs/Pose pose
    
    ================================================================================
    MSG: user_msgs/ObjBbox
    float64 x1
    float64 y1
    float64 x2
    float64 y2
    
    float64 x
    float64 y
    float64 w
    float64 h
    
    ================================================================================
    MSG: geometry_msgs/Pose
    # A representation of pose in free space, composed of position and orientation. 
    Point position
    Quaternion orientation
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    ================================================================================
    MSG: geometry_msgs/Quaternion
    # This represents an orientation in free space in quaternion form.
    
    float64 x
    float64 y
    float64 z
    float64 w
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ObjDets(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.fps !== undefined) {
      resolved.fps = msg.fps;
    }
    else {
      resolved.fps = 0
    }

    if (msg.num !== undefined) {
      resolved.num = msg.num;
    }
    else {
      resolved.num = 0
    }

    if (msg.dets !== undefined) {
      resolved.dets = new Array(msg.dets.length);
      for (let i = 0; i < resolved.dets.length; ++i) {
        resolved.dets[i] = ObjDet.Resolve(msg.dets[i]);
      }
    }
    else {
      resolved.dets = []
    }

    return resolved;
    }
};

module.exports = ObjDets;
