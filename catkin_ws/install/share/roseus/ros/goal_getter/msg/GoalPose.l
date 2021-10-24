;; Auto-generated. Do not edit!


(when (boundp 'goal_getter::GoalPose)
  (if (not (find-package "GOAL_GETTER"))
    (make-package "GOAL_GETTER"))
  (shadow 'GoalPose (find-package "GOAL_GETTER")))
(unless (find-package "GOAL_GETTER::GOALPOSE")
  (make-package "GOAL_GETTER::GOALPOSE"))

(in-package "ROS")
;;//! \htmlinclude GoalPose.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass goal_getter::GoalPose
  :super ros::object
  :slots (_goal_point _goal_normal ))

(defmethod goal_getter::GoalPose
  (:init
   (&key
    ((:goal_point __goal_point) (instance geometry_msgs::PointStamped :init))
    ((:goal_normal __goal_normal) (instance geometry_msgs::PoseStamped :init))
    )
   (send-super :init)
   (setq _goal_point __goal_point)
   (setq _goal_normal __goal_normal)
   self)
  (:goal_point
   (&rest __goal_point)
   (if (keywordp (car __goal_point))
       (send* _goal_point __goal_point)
     (progn
       (if __goal_point (setq _goal_point (car __goal_point)))
       _goal_point)))
  (:goal_normal
   (&rest __goal_normal)
   (if (keywordp (car __goal_normal))
       (send* _goal_normal __goal_normal)
     (progn
       (if __goal_normal (setq _goal_normal (car __goal_normal)))
       _goal_normal)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/PointStamped _goal_point
    (send _goal_point :serialization-length)
    ;; geometry_msgs/PoseStamped _goal_normal
    (send _goal_normal :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/PointStamped _goal_point
       (send _goal_point :serialize s)
     ;; geometry_msgs/PoseStamped _goal_normal
       (send _goal_normal :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/PointStamped _goal_point
     (send _goal_point :deserialize buf ptr-) (incf ptr- (send _goal_point :serialization-length))
   ;; geometry_msgs/PoseStamped _goal_normal
     (send _goal_normal :deserialize buf ptr-) (incf ptr- (send _goal_normal :serialization-length))
   ;;
   self)
  )

(setf (get goal_getter::GoalPose :md5sum-) "eaffae3e07b3d469bde28c34c52087ab")
(setf (get goal_getter::GoalPose :datatype-) "goal_getter/GoalPose")
(setf (get goal_getter::GoalPose :definition-)
      "geometry_msgs/PointStamped goal_point
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

")



(provide :goal_getter/GoalPose "eaffae3e07b3d469bde28c34c52087ab")


