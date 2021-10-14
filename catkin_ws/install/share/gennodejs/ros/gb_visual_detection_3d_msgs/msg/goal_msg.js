// Auto-generated. Do not edit!

// (in-package gb_visual_detection_3d_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class goal_msg {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.Class = null;
      this.x = null;
      this.y = null;
      this.z = null;
      this.normal_x = null;
      this.normal_y = null;
      this.normal_z = null;
    }
    else {
      if (initObj.hasOwnProperty('Class')) {
        this.Class = initObj.Class
      }
      else {
        this.Class = '';
      }
      if (initObj.hasOwnProperty('x')) {
        this.x = initObj.x
      }
      else {
        this.x = 0.0;
      }
      if (initObj.hasOwnProperty('y')) {
        this.y = initObj.y
      }
      else {
        this.y = 0.0;
      }
      if (initObj.hasOwnProperty('z')) {
        this.z = initObj.z
      }
      else {
        this.z = 0.0;
      }
      if (initObj.hasOwnProperty('normal_x')) {
        this.normal_x = initObj.normal_x
      }
      else {
        this.normal_x = 0.0;
      }
      if (initObj.hasOwnProperty('normal_y')) {
        this.normal_y = initObj.normal_y
      }
      else {
        this.normal_y = 0.0;
      }
      if (initObj.hasOwnProperty('normal_z')) {
        this.normal_z = initObj.normal_z
      }
      else {
        this.normal_z = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type goal_msg
    // Serialize message field [Class]
    bufferOffset = _serializer.string(obj.Class, buffer, bufferOffset);
    // Serialize message field [x]
    bufferOffset = _serializer.float64(obj.x, buffer, bufferOffset);
    // Serialize message field [y]
    bufferOffset = _serializer.float64(obj.y, buffer, bufferOffset);
    // Serialize message field [z]
    bufferOffset = _serializer.float64(obj.z, buffer, bufferOffset);
    // Serialize message field [normal_x]
    bufferOffset = _serializer.float64(obj.normal_x, buffer, bufferOffset);
    // Serialize message field [normal_y]
    bufferOffset = _serializer.float64(obj.normal_y, buffer, bufferOffset);
    // Serialize message field [normal_z]
    bufferOffset = _serializer.float64(obj.normal_z, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type goal_msg
    let len;
    let data = new goal_msg(null);
    // Deserialize message field [Class]
    data.Class = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [x]
    data.x = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [y]
    data.y = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [z]
    data.z = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [normal_x]
    data.normal_x = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [normal_y]
    data.normal_y = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [normal_z]
    data.normal_z = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.Class.length;
    return length + 52;
  }

  static datatype() {
    // Returns string type for a message object
    return 'gb_visual_detection_3d_msgs/goal_msg';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a14534629947e0e1a95981852d661e1b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string Class
    float64 x
    float64 y
    float64 z
    float64 normal_x
    float64 normal_y
    float64 normal_z
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new goal_msg(null);
    if (msg.Class !== undefined) {
      resolved.Class = msg.Class;
    }
    else {
      resolved.Class = ''
    }

    if (msg.x !== undefined) {
      resolved.x = msg.x;
    }
    else {
      resolved.x = 0.0
    }

    if (msg.y !== undefined) {
      resolved.y = msg.y;
    }
    else {
      resolved.y = 0.0
    }

    if (msg.z !== undefined) {
      resolved.z = msg.z;
    }
    else {
      resolved.z = 0.0
    }

    if (msg.normal_x !== undefined) {
      resolved.normal_x = msg.normal_x;
    }
    else {
      resolved.normal_x = 0.0
    }

    if (msg.normal_y !== undefined) {
      resolved.normal_y = msg.normal_y;
    }
    else {
      resolved.normal_y = 0.0
    }

    if (msg.normal_z !== undefined) {
      resolved.normal_z = msg.normal_z;
    }
    else {
      resolved.normal_z = 0.0
    }

    return resolved;
    }
};

module.exports = goal_msg;
