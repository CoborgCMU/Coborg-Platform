;; Auto-generated. Do not edit!


(when (boundp 'franka_interface_msgs::ExecuteSkillActionGoal)
  (if (not (find-package "FRANKA_INTERFACE_MSGS"))
    (make-package "FRANKA_INTERFACE_MSGS"))
  (shadow 'ExecuteSkillActionGoal (find-package "FRANKA_INTERFACE_MSGS")))
(unless (find-package "FRANKA_INTERFACE_MSGS::EXECUTESKILLACTIONGOAL")
  (make-package "FRANKA_INTERFACE_MSGS::EXECUTESKILLACTIONGOAL"))

(in-package "ROS")
;;//! \htmlinclude ExecuteSkillActionGoal.msg.html
(if (not (find-package "ACTIONLIB_MSGS"))
  (ros::roseus-add-msgs "actionlib_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass franka_interface_msgs::ExecuteSkillActionGoal
  :super ros::object
  :slots (_header _goal_id _goal ))

(defmethod franka_interface_msgs::ExecuteSkillActionGoal
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:goal_id __goal_id) (instance actionlib_msgs::GoalID :init))
    ((:goal __goal) (instance franka_interface_msgs::ExecuteSkillGoal :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _goal_id __goal_id)
   (setq _goal __goal)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:goal_id
   (&rest __goal_id)
   (if (keywordp (car __goal_id))
       (send* _goal_id __goal_id)
     (progn
       (if __goal_id (setq _goal_id (car __goal_id)))
       _goal_id)))
  (:goal
   (&rest __goal)
   (if (keywordp (car __goal))
       (send* _goal __goal)
     (progn
       (if __goal (setq _goal (car __goal)))
       _goal)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; actionlib_msgs/GoalID _goal_id
    (send _goal_id :serialization-length)
    ;; franka_interface_msgs/ExecuteSkillGoal _goal
    (send _goal :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; actionlib_msgs/GoalID _goal_id
       (send _goal_id :serialize s)
     ;; franka_interface_msgs/ExecuteSkillGoal _goal
       (send _goal :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; actionlib_msgs/GoalID _goal_id
     (send _goal_id :deserialize buf ptr-) (incf ptr- (send _goal_id :serialization-length))
   ;; franka_interface_msgs/ExecuteSkillGoal _goal
     (send _goal :deserialize buf ptr-) (incf ptr- (send _goal :serialization-length))
   ;;
   self)
  )

(setf (get franka_interface_msgs::ExecuteSkillActionGoal :md5sum-) "9010c44dae6748cc80ceac2c2cce6e13")
(setf (get franka_interface_msgs::ExecuteSkillActionGoal :datatype-) "franka_interface_msgs/ExecuteSkillActionGoal")
(setf (get franka_interface_msgs::ExecuteSkillActionGoal :definition-)
      "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======

Header header
actionlib_msgs/GoalID goal_id
ExecuteSkillGoal goal

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
MSG: actionlib_msgs/GoalID
# The stamp should store the time at which this goal was requested.
# It is used by an action server when it tries to preempt all
# goals that were requested before a certain time
time stamp

# The id provides a way to associate feedback and
# result message with specific goal requests. The id
# specified must be unique.
string id


================================================================================
MSG: franka_interface_msgs/ExecuteSkillGoal
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
uint8 skill_type
string skill_description
uint8 meta_skill_type
int64 meta_skill_id

# Sensor topic to subscribe to
string[] sensor_topics
uint64[] sensor_value_sizes
float64[] initial_sensor_values

# traj gen
uint8 trajectory_generator_type
int32 trajectory_generator_param_data_size
uint8[] trajectory_generator_param_data

# fbc
uint8 feedback_controller_type
int32 feedback_controller_param_data_size
uint8[] feedback_controller_param_data

# termination
uint8 termination_handler_type
int32 termination_handler_param_data_size
uint8[] termination_handler_param_data 

# timer
uint8 timer_type
int32 num_timer_params
uint8[] timer_params

")



(provide :franka_interface_msgs/ExecuteSkillActionGoal "9010c44dae6748cc80ceac2c2cce6e13")

