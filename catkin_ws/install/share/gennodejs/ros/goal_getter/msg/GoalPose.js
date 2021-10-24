// Auto-generated. Do not edit!

// (in-package goal_getter.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class GoalPose {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.goal_point = null;
      this.goal_normal = null;
    }
    else {
      if (initObj.hasOwnProperty('goal_point')) {
        this.goal_point = initObj.goal_point
      }
      else {
        this.goal_point = new geometry_msgs.msg.PointStamped();
      }
      if (initObj.hasOwnProperty('goal_normal')) {
        this.goal_normal = initObj.goal_normal
      }
      else {
        this.goal_normal = new geometry_msgs.msg.PoseStamped();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GoalPose
    // Serialize message field [goal_point]
    bufferOffset = geometry_msgs.msg.PointStamped.serialize(obj.goal_point, buffer, bufferOffset);
    // Serialize message field [goal_normal]
    bufferOffset = geometry_msgs.msg.PoseStamped.serialize(obj.goal_normal, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GoalPose
    let len;
    let data = new GoalPose(null);
    // Deserialize message field [goal_point]
    data.goal_point = geometry_msgs.msg.PointStamped.deserialize(buffer, bufferOffset);
    // Deserialize message field [goal_normal]
    data.goal_normal = geometry_msgs.msg.PoseStamped.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += geometry_msgs.msg.PointStamped.getMessageSize(object.goal_point);
    length += geometry_msgs.msg.PoseStamped.getMessageSize(object.goal_normal);
    return length;
  }

  static datatype() {
    // Returns string type for a message object
    return 'goal_getter/GoalPose';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'eaffae3e07b3d469bde28c34c52087ab';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    geometry_msgs/PointStamped goal_point
    geometry_msgs/PoseStamped goal_normal
    
    ================================================================================
    MSG: geometry_msgs/PointStamped
    # This represents a Point with reference coordinate frame and timestamp
    Header header
    Point point
    
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
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    ================================================================================
    MSG: geometry_msgs/PoseStamped
    # A Pose with reference coordinate frame and timestamp
    Header header
    Pose pose
    
    ================================================================================
    MSG: geometry_msgs/Pose
    # A representation of pose in free space, composed of position and orientation. 
    Point position
    Quaternion orientation
    
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
    const resolved = new GoalPose(null);
    if (msg.goal_point !== undefined) {
      resolved.goal_point = geometry_msgs.msg.PointStamped.Resolve(msg.goal_point)
    }
    else {
      resolved.goal_point = new geometry_msgs.msg.PointStamped()
    }

    if (msg.goal_normal !== undefined) {
      resolved.goal_normal = geometry_msgs.msg.PoseStamped.Resolve(msg.goal_normal)
    }
    else {
      resolved.goal_normal = new geometry_msgs.msg.PoseStamped()
    }

    return resolved;
    }
};

module.exports = GoalPose;
