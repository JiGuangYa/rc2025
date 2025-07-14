// Auto-generated. Do not edit!

// (in-package user_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let ObjBbox = require('./ObjBbox.js');
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class ObjDet {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.index = null;
      this.class_id = null;
      this.label = null;
      this.score = null;
      this.bbox = null;
      this.pose = null;
    }
    else {
      if (initObj.hasOwnProperty('index')) {
        this.index = initObj.index
      }
      else {
        this.index = 0;
      }
      if (initObj.hasOwnProperty('class_id')) {
        this.class_id = initObj.class_id
      }
      else {
        this.class_id = 0;
      }
      if (initObj.hasOwnProperty('label')) {
        this.label = initObj.label
      }
      else {
        this.label = '';
      }
      if (initObj.hasOwnProperty('score')) {
        this.score = initObj.score
      }
      else {
        this.score = 0.0;
      }
      if (initObj.hasOwnProperty('bbox')) {
        this.bbox = initObj.bbox
      }
      else {
        this.bbox = new ObjBbox();
      }
      if (initObj.hasOwnProperty('pose')) {
        this.pose = initObj.pose
      }
      else {
        this.pose = new geometry_msgs.msg.Pose();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ObjDet
    // Serialize message field [index]
    bufferOffset = _serializer.uint16(obj.index, buffer, bufferOffset);
    // Serialize message field [class_id]
    bufferOffset = _serializer.uint8(obj.class_id, buffer, bufferOffset);
    // Serialize message field [label]
    bufferOffset = _serializer.string(obj.label, buffer, bufferOffset);
    // Serialize message field [score]
    bufferOffset = _serializer.float64(obj.score, buffer, bufferOffset);
    // Serialize message field [bbox]
    bufferOffset = ObjBbox.serialize(obj.bbox, buffer, bufferOffset);
    // Serialize message field [pose]
    bufferOffset = geometry_msgs.msg.Pose.serialize(obj.pose, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ObjDet
    let len;
    let data = new ObjDet(null);
    // Deserialize message field [index]
    data.index = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [class_id]
    data.class_id = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [label]
    data.label = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [score]
    data.score = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [bbox]
    data.bbox = ObjBbox.deserialize(buffer, bufferOffset);
    // Deserialize message field [pose]
    data.pose = geometry_msgs.msg.Pose.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.label);
    return length + 135;
  }

  static datatype() {
    // Returns string type for a message object
    return 'user_msgs/ObjDet';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '38284f8c06edb8f35a2f0a300b2226fc';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    const resolved = new ObjDet(null);
    if (msg.index !== undefined) {
      resolved.index = msg.index;
    }
    else {
      resolved.index = 0
    }

    if (msg.class_id !== undefined) {
      resolved.class_id = msg.class_id;
    }
    else {
      resolved.class_id = 0
    }

    if (msg.label !== undefined) {
      resolved.label = msg.label;
    }
    else {
      resolved.label = ''
    }

    if (msg.score !== undefined) {
      resolved.score = msg.score;
    }
    else {
      resolved.score = 0.0
    }

    if (msg.bbox !== undefined) {
      resolved.bbox = ObjBbox.Resolve(msg.bbox)
    }
    else {
      resolved.bbox = new ObjBbox()
    }

    if (msg.pose !== undefined) {
      resolved.pose = geometry_msgs.msg.Pose.Resolve(msg.pose)
    }
    else {
      resolved.pose = new geometry_msgs.msg.Pose()
    }

    return resolved;
    }
};

module.exports = ObjDet;
