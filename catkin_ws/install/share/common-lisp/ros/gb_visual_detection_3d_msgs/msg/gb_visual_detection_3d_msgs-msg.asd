
(cl:in-package :asdf)

(defsystem "gb_visual_detection_3d_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "BoundingBox3d" :depends-on ("_package_BoundingBox3d"))
    (:file "_package_BoundingBox3d" :depends-on ("_package"))
    (:file "BoundingBoxes3d" :depends-on ("_package_BoundingBoxes3d"))
    (:file "_package_BoundingBoxes3d" :depends-on ("_package"))
    (:file "goal_msg" :depends-on ("_package_goal_msg"))
    (:file "_package_goal_msg" :depends-on ("_package"))
  ))