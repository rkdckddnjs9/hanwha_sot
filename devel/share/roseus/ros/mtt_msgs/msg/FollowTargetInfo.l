;; Auto-generated. Do not edit!


(when (boundp 'mtt_msgs::FollowTargetInfo)
  (if (not (find-package "MTT_MSGS"))
    (make-package "MTT_MSGS"))
  (shadow 'FollowTargetInfo (find-package "MTT_MSGS")))
(unless (find-package "MTT_MSGS::FOLLOWTARGETINFO")
  (make-package "MTT_MSGS::FOLLOWTARGETINFO"))

(in-package "ROS")
;;//! \htmlinclude FollowTargetInfo.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass mtt_msgs::FollowTargetInfo
  :super ros::object
  :slots (_header _id _object_class _score _pose _size _velocity ))

(defmethod mtt_msgs::FollowTargetInfo
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:id __id) (instance std_msgs::Int32 :init))
    ((:object_class __object_class) (instance std_msgs::Int8 :init))
    ((:score __score) (instance std_msgs::Float32 :init))
    ((:pose __pose) (instance geometry_msgs::Pose :init))
    ((:size __size) (instance geometry_msgs::Point :init))
    ((:velocity __velocity) (instance geometry_msgs::Point :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _id __id)
   (setq _object_class __object_class)
   (setq _score __score)
   (setq _pose __pose)
   (setq _size __size)
   (setq _velocity __velocity)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:id
   (&rest __id)
   (if (keywordp (car __id))
       (send* _id __id)
     (progn
       (if __id (setq _id (car __id)))
       _id)))
  (:object_class
   (&rest __object_class)
   (if (keywordp (car __object_class))
       (send* _object_class __object_class)
     (progn
       (if __object_class (setq _object_class (car __object_class)))
       _object_class)))
  (:score
   (&rest __score)
   (if (keywordp (car __score))
       (send* _score __score)
     (progn
       (if __score (setq _score (car __score)))
       _score)))
  (:pose
   (&rest __pose)
   (if (keywordp (car __pose))
       (send* _pose __pose)
     (progn
       (if __pose (setq _pose (car __pose)))
       _pose)))
  (:size
   (&rest __size)
   (if (keywordp (car __size))
       (send* _size __size)
     (progn
       (if __size (setq _size (car __size)))
       _size)))
  (:velocity
   (&rest __velocity)
   (if (keywordp (car __velocity))
       (send* _velocity __velocity)
     (progn
       (if __velocity (setq _velocity (car __velocity)))
       _velocity)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; std_msgs/Int32 _id
    (send _id :serialization-length)
    ;; std_msgs/Int8 _object_class
    (send _object_class :serialization-length)
    ;; std_msgs/Float32 _score
    (send _score :serialization-length)
    ;; geometry_msgs/Pose _pose
    (send _pose :serialization-length)
    ;; geometry_msgs/Point _size
    (send _size :serialization-length)
    ;; geometry_msgs/Point _velocity
    (send _velocity :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; std_msgs/Int32 _id
       (send _id :serialize s)
     ;; std_msgs/Int8 _object_class
       (send _object_class :serialize s)
     ;; std_msgs/Float32 _score
       (send _score :serialize s)
     ;; geometry_msgs/Pose _pose
       (send _pose :serialize s)
     ;; geometry_msgs/Point _size
       (send _size :serialize s)
     ;; geometry_msgs/Point _velocity
       (send _velocity :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; std_msgs/Int32 _id
     (send _id :deserialize buf ptr-) (incf ptr- (send _id :serialization-length))
   ;; std_msgs/Int8 _object_class
     (send _object_class :deserialize buf ptr-) (incf ptr- (send _object_class :serialization-length))
   ;; std_msgs/Float32 _score
     (send _score :deserialize buf ptr-) (incf ptr- (send _score :serialization-length))
   ;; geometry_msgs/Pose _pose
     (send _pose :deserialize buf ptr-) (incf ptr- (send _pose :serialization-length))
   ;; geometry_msgs/Point _size
     (send _size :deserialize buf ptr-) (incf ptr- (send _size :serialization-length))
   ;; geometry_msgs/Point _velocity
     (send _velocity :deserialize buf ptr-) (incf ptr- (send _velocity :serialization-length))
   ;;
   self)
  )

(setf (get mtt_msgs::FollowTargetInfo :md5sum-) "f662c869093ee1a8404764360b3f4101")
(setf (get mtt_msgs::FollowTargetInfo :datatype-) "mtt_msgs/FollowTargetInfo")
(setf (get mtt_msgs::FollowTargetInfo :definition-)
      "std_msgs/Header header
std_msgs/Int32 id
std_msgs/Int8 object_class
std_msgs/Float32 score
geometry_msgs/Pose pose
geometry_msgs/Point size
geometry_msgs/Point velocity
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
MSG: std_msgs/Float32
float32 data
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

")



(provide :mtt_msgs/FollowTargetInfo "f662c869093ee1a8404764360b3f4101")


