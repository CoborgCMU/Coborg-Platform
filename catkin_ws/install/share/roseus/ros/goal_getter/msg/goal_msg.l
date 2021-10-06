;; Auto-generated. Do not edit!


(when (boundp 'goal_getter::goal_msg)
  (if (not (find-package "GOAL_GETTER"))
    (make-package "GOAL_GETTER"))
  (shadow 'goal_msg (find-package "GOAL_GETTER")))
(unless (find-package "GOAL_GETTER::GOAL_MSG")
  (make-package "GOAL_GETTER::GOAL_MSG"))

(in-package "ROS")
;;//! \htmlinclude goal_msg.msg.html


(defclass goal_getter::goal_msg
  :super ros::object
  :slots (_Class _x _y _z _normal_x _normal_y _normal_z ))

(defmethod goal_getter::goal_msg
  (:init
   (&key
    ((:Class __Class) "")
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:z __z) 0.0)
    ((:normal_x __normal_x) 0.0)
    ((:normal_y __normal_y) 0.0)
    ((:normal_z __normal_z) 0.0)
    )
   (send-super :init)
   (setq _Class (string __Class))
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _z (float __z))
   (setq _normal_x (float __normal_x))
   (setq _normal_y (float __normal_y))
   (setq _normal_z (float __normal_z))
   self)
  (:Class
   (&optional __Class)
   (if __Class (setq _Class __Class)) _Class)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:z
   (&optional __z)
   (if __z (setq _z __z)) _z)
  (:normal_x
   (&optional __normal_x)
   (if __normal_x (setq _normal_x __normal_x)) _normal_x)
  (:normal_y
   (&optional __normal_y)
   (if __normal_y (setq _normal_y __normal_y)) _normal_y)
  (:normal_z
   (&optional __normal_z)
   (if __normal_z (setq _normal_z __normal_z)) _normal_z)
  (:serialization-length
   ()
   (+
    ;; string _Class
    4 (length _Class)
    ;; float64 _x
    8
    ;; float64 _y
    8
    ;; float64 _z
    8
    ;; float64 _normal_x
    8
    ;; float64 _normal_y
    8
    ;; float64 _normal_z
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _Class
       (write-long (length _Class) s) (princ _Class s)
     ;; float64 _x
       (sys::poke _x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _y
       (sys::poke _y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _z
       (sys::poke _z (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _normal_x
       (sys::poke _normal_x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _normal_y
       (sys::poke _normal_y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _normal_z
       (sys::poke _normal_z (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _Class
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _Class (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float64 _x
     (setq _x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _y
     (setq _y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _z
     (setq _z (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _normal_x
     (setq _normal_x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _normal_y
     (setq _normal_y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _normal_z
     (setq _normal_z (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get goal_getter::goal_msg :md5sum-) "a14534629947e0e1a95981852d661e1b")
(setf (get goal_getter::goal_msg :datatype-) "goal_getter/goal_msg")
(setf (get goal_getter::goal_msg :definition-)
      "string Class
float64 x
float64 y
float64 z
float64 normal_x
float64 normal_y
float64 normal_z

")



(provide :goal_getter/goal_msg "a14534629947e0e1a95981852d661e1b")


