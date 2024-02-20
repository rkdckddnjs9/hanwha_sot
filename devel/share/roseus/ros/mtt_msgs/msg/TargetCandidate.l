;; Auto-generated. Do not edit!


(when (boundp 'mtt_msgs::TargetCandidate)
  (if (not (find-package "MTT_MSGS"))
    (make-package "MTT_MSGS"))
  (shadow 'TargetCandidate (find-package "MTT_MSGS")))
(unless (find-package "MTT_MSGS::TARGETCANDIDATE")
  (make-package "MTT_MSGS::TARGETCANDIDATE"))

(in-package "ROS")
;;//! \htmlinclude TargetCandidate.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass mtt_msgs::TargetCandidate
  :super ros::object
  :slots (_header _id_1 _class_1 _position_1 _id_2 _class_2 _position_2 _id_3 _class_3 _position_3 _id_4 _class_4 _position_4 _id_5 _class_5 _position_5 ))

(defmethod mtt_msgs::TargetCandidate
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:id_1 __id_1) (instance std_msgs::Int32 :init))
    ((:class_1 __class_1) (instance std_msgs::Int8 :init))
    ((:position_1 __position_1) (instance geometry_msgs::Point :init))
    ((:id_2 __id_2) (instance std_msgs::Int32 :init))
    ((:class_2 __class_2) (instance std_msgs::Int8 :init))
    ((:position_2 __position_2) (instance geometry_msgs::Point :init))
    ((:id_3 __id_3) (instance std_msgs::Int32 :init))
    ((:class_3 __class_3) (instance std_msgs::Int8 :init))
    ((:position_3 __position_3) (instance geometry_msgs::Point :init))
    ((:id_4 __id_4) (instance std_msgs::Int32 :init))
    ((:class_4 __class_4) (instance std_msgs::Int8 :init))
    ((:position_4 __position_4) (instance geometry_msgs::Point :init))
    ((:id_5 __id_5) (instance std_msgs::Int32 :init))
    ((:class_5 __class_5) (instance std_msgs::Int8 :init))
    ((:position_5 __position_5) (instance geometry_msgs::Point :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _id_1 __id_1)
   (setq _class_1 __class_1)
   (setq _position_1 __position_1)
   (setq _id_2 __id_2)
   (setq _class_2 __class_2)
   (setq _position_2 __position_2)
   (setq _id_3 __id_3)
   (setq _class_3 __class_3)
   (setq _position_3 __position_3)
   (setq _id_4 __id_4)
   (setq _class_4 __class_4)
   (setq _position_4 __position_4)
   (setq _id_5 __id_5)
   (setq _class_5 __class_5)
   (setq _position_5 __position_5)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:id_1
   (&rest __id_1)
   (if (keywordp (car __id_1))
       (send* _id_1 __id_1)
     (progn
       (if __id_1 (setq _id_1 (car __id_1)))
       _id_1)))
  (:class_1
   (&rest __class_1)
   (if (keywordp (car __class_1))
       (send* _class_1 __class_1)
     (progn
       (if __class_1 (setq _class_1 (car __class_1)))
       _class_1)))
  (:position_1
   (&rest __position_1)
   (if (keywordp (car __position_1))
       (send* _position_1 __position_1)
     (progn
       (if __position_1 (setq _position_1 (car __position_1)))
       _position_1)))
  (:id_2
   (&rest __id_2)
   (if (keywordp (car __id_2))
       (send* _id_2 __id_2)
     (progn
       (if __id_2 (setq _id_2 (car __id_2)))
       _id_2)))
  (:class_2
   (&rest __class_2)
   (if (keywordp (car __class_2))
       (send* _class_2 __class_2)
     (progn
       (if __class_2 (setq _class_2 (car __class_2)))
       _class_2)))
  (:position_2
   (&rest __position_2)
   (if (keywordp (car __position_2))
       (send* _position_2 __position_2)
     (progn
       (if __position_2 (setq _position_2 (car __position_2)))
       _position_2)))
  (:id_3
   (&rest __id_3)
   (if (keywordp (car __id_3))
       (send* _id_3 __id_3)
     (progn
       (if __id_3 (setq _id_3 (car __id_3)))
       _id_3)))
  (:class_3
   (&rest __class_3)
   (if (keywordp (car __class_3))
       (send* _class_3 __class_3)
     (progn
       (if __class_3 (setq _class_3 (car __class_3)))
       _class_3)))
  (:position_3
   (&rest __position_3)
   (if (keywordp (car __position_3))
       (send* _position_3 __position_3)
     (progn
       (if __position_3 (setq _position_3 (car __position_3)))
       _position_3)))
  (:id_4
   (&rest __id_4)
   (if (keywordp (car __id_4))
       (send* _id_4 __id_4)
     (progn
       (if __id_4 (setq _id_4 (car __id_4)))
       _id_4)))
  (:class_4
   (&rest __class_4)
   (if (keywordp (car __class_4))
       (send* _class_4 __class_4)
     (progn
       (if __class_4 (setq _class_4 (car __class_4)))
       _class_4)))
  (:position_4
   (&rest __position_4)
   (if (keywordp (car __position_4))
       (send* _position_4 __position_4)
     (progn
       (if __position_4 (setq _position_4 (car __position_4)))
       _position_4)))
  (:id_5
   (&rest __id_5)
   (if (keywordp (car __id_5))
       (send* _id_5 __id_5)
     (progn
       (if __id_5 (setq _id_5 (car __id_5)))
       _id_5)))
  (:class_5
   (&rest __class_5)
   (if (keywordp (car __class_5))
       (send* _class_5 __class_5)
     (progn
       (if __class_5 (setq _class_5 (car __class_5)))
       _class_5)))
  (:position_5
   (&rest __position_5)
   (if (keywordp (car __position_5))
       (send* _position_5 __position_5)
     (progn
       (if __position_5 (setq _position_5 (car __position_5)))
       _position_5)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; std_msgs/Int32 _id_1
    (send _id_1 :serialization-length)
    ;; std_msgs/Int8 _class_1
    (send _class_1 :serialization-length)
    ;; geometry_msgs/Point _position_1
    (send _position_1 :serialization-length)
    ;; std_msgs/Int32 _id_2
    (send _id_2 :serialization-length)
    ;; std_msgs/Int8 _class_2
    (send _class_2 :serialization-length)
    ;; geometry_msgs/Point _position_2
    (send _position_2 :serialization-length)
    ;; std_msgs/Int32 _id_3
    (send _id_3 :serialization-length)
    ;; std_msgs/Int8 _class_3
    (send _class_3 :serialization-length)
    ;; geometry_msgs/Point _position_3
    (send _position_3 :serialization-length)
    ;; std_msgs/Int32 _id_4
    (send _id_4 :serialization-length)
    ;; std_msgs/Int8 _class_4
    (send _class_4 :serialization-length)
    ;; geometry_msgs/Point _position_4
    (send _position_4 :serialization-length)
    ;; std_msgs/Int32 _id_5
    (send _id_5 :serialization-length)
    ;; std_msgs/Int8 _class_5
    (send _class_5 :serialization-length)
    ;; geometry_msgs/Point _position_5
    (send _position_5 :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; std_msgs/Int32 _id_1
       (send _id_1 :serialize s)
     ;; std_msgs/Int8 _class_1
       (send _class_1 :serialize s)
     ;; geometry_msgs/Point _position_1
       (send _position_1 :serialize s)
     ;; std_msgs/Int32 _id_2
       (send _id_2 :serialize s)
     ;; std_msgs/Int8 _class_2
       (send _class_2 :serialize s)
     ;; geometry_msgs/Point _position_2
       (send _position_2 :serialize s)
     ;; std_msgs/Int32 _id_3
       (send _id_3 :serialize s)
     ;; std_msgs/Int8 _class_3
       (send _class_3 :serialize s)
     ;; geometry_msgs/Point _position_3
       (send _position_3 :serialize s)
     ;; std_msgs/Int32 _id_4
       (send _id_4 :serialize s)
     ;; std_msgs/Int8 _class_4
       (send _class_4 :serialize s)
     ;; geometry_msgs/Point _position_4
       (send _position_4 :serialize s)
     ;; std_msgs/Int32 _id_5
       (send _id_5 :serialize s)
     ;; std_msgs/Int8 _class_5
       (send _class_5 :serialize s)
     ;; geometry_msgs/Point _position_5
       (send _position_5 :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; std_msgs/Int32 _id_1
     (send _id_1 :deserialize buf ptr-) (incf ptr- (send _id_1 :serialization-length))
   ;; std_msgs/Int8 _class_1
     (send _class_1 :deserialize buf ptr-) (incf ptr- (send _class_1 :serialization-length))
   ;; geometry_msgs/Point _position_1
     (send _position_1 :deserialize buf ptr-) (incf ptr- (send _position_1 :serialization-length))
   ;; std_msgs/Int32 _id_2
     (send _id_2 :deserialize buf ptr-) (incf ptr- (send _id_2 :serialization-length))
   ;; std_msgs/Int8 _class_2
     (send _class_2 :deserialize buf ptr-) (incf ptr- (send _class_2 :serialization-length))
   ;; geometry_msgs/Point _position_2
     (send _position_2 :deserialize buf ptr-) (incf ptr- (send _position_2 :serialization-length))
   ;; std_msgs/Int32 _id_3
     (send _id_3 :deserialize buf ptr-) (incf ptr- (send _id_3 :serialization-length))
   ;; std_msgs/Int8 _class_3
     (send _class_3 :deserialize buf ptr-) (incf ptr- (send _class_3 :serialization-length))
   ;; geometry_msgs/Point _position_3
     (send _position_3 :deserialize buf ptr-) (incf ptr- (send _position_3 :serialization-length))
   ;; std_msgs/Int32 _id_4
     (send _id_4 :deserialize buf ptr-) (incf ptr- (send _id_4 :serialization-length))
   ;; std_msgs/Int8 _class_4
     (send _class_4 :deserialize buf ptr-) (incf ptr- (send _class_4 :serialization-length))
   ;; geometry_msgs/Point _position_4
     (send _position_4 :deserialize buf ptr-) (incf ptr- (send _position_4 :serialization-length))
   ;; std_msgs/Int32 _id_5
     (send _id_5 :deserialize buf ptr-) (incf ptr- (send _id_5 :serialization-length))
   ;; std_msgs/Int8 _class_5
     (send _class_5 :deserialize buf ptr-) (incf ptr- (send _class_5 :serialization-length))
   ;; geometry_msgs/Point _position_5
     (send _position_5 :deserialize buf ptr-) (incf ptr- (send _position_5 :serialization-length))
   ;;
   self)
  )

(setf (get mtt_msgs::TargetCandidate :md5sum-) "fc377ddc3099b4c637aba5f3b76231f6")
(setf (get mtt_msgs::TargetCandidate :datatype-) "mtt_msgs/TargetCandidate")
(setf (get mtt_msgs::TargetCandidate :definition-)
      "std_msgs/Header header
std_msgs/Int32 id_1
std_msgs/Int8 class_1
geometry_msgs/Point position_1
std_msgs/Int32 id_2
std_msgs/Int8 class_2
geometry_msgs/Point position_2
std_msgs/Int32 id_3
std_msgs/Int8 class_3
geometry_msgs/Point position_3
std_msgs/Int32 id_4
std_msgs/Int8 class_4
geometry_msgs/Point position_4
std_msgs/Int32 id_5
std_msgs/Int8 class_5
geometry_msgs/Point position_5

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
MSG: std_msgs/Int32
int32 data
================================================================================
MSG: std_msgs/Int8
int8 data

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :mtt_msgs/TargetCandidate "fc377ddc3099b4c637aba5f3b76231f6")


