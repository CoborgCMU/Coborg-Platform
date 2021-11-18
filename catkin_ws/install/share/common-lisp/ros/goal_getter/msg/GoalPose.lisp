; Auto-generated. Do not edit!


(cl:in-package goal_getter-msg)


;//! \htmlinclude GoalPose.msg.html

(cl:defclass <GoalPose> (roslisp-msg-protocol:ros-message)
  ((goal_point
    :reader goal_point
    :initarg :goal_point
    :type geometry_msgs-msg:PointStamped
    :initform (cl:make-instance 'geometry_msgs-msg:PointStamped))
   (goal_normal
    :reader goal_normal
    :initarg :goal_normal
    :type geometry_msgs-msg:PoseStamped
    :initform (cl:make-instance 'geometry_msgs-msg:PoseStamped)))
)

(cl:defclass GoalPose (<GoalPose>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GoalPose>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GoalPose)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name goal_getter-msg:<GoalPose> is deprecated: use goal_getter-msg:GoalPose instead.")))

(cl:ensure-generic-function 'goal_point-val :lambda-list '(m))
(cl:defmethod goal_point-val ((m <GoalPose>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader goal_getter-msg:goal_point-val is deprecated.  Use goal_getter-msg:goal_point instead.")
  (goal_point m))

(cl:ensure-generic-function 'goal_normal-val :lambda-list '(m))
(cl:defmethod goal_normal-val ((m <GoalPose>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader goal_getter-msg:goal_normal-val is deprecated.  Use goal_getter-msg:goal_normal instead.")
  (goal_normal m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GoalPose>) ostream)
  "Serializes a message object of type '<GoalPose>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'goal_point) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'goal_normal) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GoalPose>) istream)
  "Deserializes a message object of type '<GoalPose>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'goal_point) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'goal_normal) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GoalPose>)))
  "Returns string type for a message object of type '<GoalPose>"
  "goal_getter/GoalPose")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GoalPose)))
  "Returns string type for a message object of type 'GoalPose"
  "goal_getter/GoalPose")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GoalPose>)))
  "Returns md5sum for a message object of type '<GoalPose>"
  "eaffae3e07b3d469bde28c34c52087ab")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GoalPose)))
  "Returns md5sum for a message object of type 'GoalPose"
  "eaffae3e07b3d469bde28c34c52087ab")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GoalPose>)))
  "Returns full string definition for message of type '<GoalPose>"
  (cl:format cl:nil "geometry_msgs/PointStamped goal_point~%geometry_msgs/PoseStamped goal_normal~%~%================================================================================~%MSG: geometry_msgs/PointStamped~%# This represents a Point with reference coordinate frame and timestamp~%Header header~%Point point~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GoalPose)))
  "Returns full string definition for message of type 'GoalPose"
  (cl:format cl:nil "geometry_msgs/PointStamped goal_point~%geometry_msgs/PoseStamped goal_normal~%~%================================================================================~%MSG: geometry_msgs/PointStamped~%# This represents a Point with reference coordinate frame and timestamp~%Header header~%Point point~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GoalPose>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'goal_point))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'goal_normal))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GoalPose>))
  "Converts a ROS message object to a list"
  (cl:list 'GoalPose
    (cl:cons ':goal_point (goal_point msg))
    (cl:cons ':goal_normal (goal_normal msg))
))
