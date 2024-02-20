; Auto-generated. Do not edit!


(cl:in-package mtt_msgs-msg)


;//! \htmlinclude FollowTargetInfo.msg.html

(cl:defclass <FollowTargetInfo> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (id
    :reader id
    :initarg :id
    :type std_msgs-msg:Int32
    :initform (cl:make-instance 'std_msgs-msg:Int32))
   (object_class
    :reader object_class
    :initarg :object_class
    :type std_msgs-msg:Int8
    :initform (cl:make-instance 'std_msgs-msg:Int8))
   (score
    :reader score
    :initarg :score
    :type std_msgs-msg:Float32
    :initform (cl:make-instance 'std_msgs-msg:Float32))
   (pose
    :reader pose
    :initarg :pose
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose))
   (size
    :reader size
    :initarg :size
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (velocity
    :reader velocity
    :initarg :velocity
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point)))
)

(cl:defclass FollowTargetInfo (<FollowTargetInfo>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FollowTargetInfo>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FollowTargetInfo)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mtt_msgs-msg:<FollowTargetInfo> is deprecated: use mtt_msgs-msg:FollowTargetInfo instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <FollowTargetInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mtt_msgs-msg:header-val is deprecated.  Use mtt_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <FollowTargetInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mtt_msgs-msg:id-val is deprecated.  Use mtt_msgs-msg:id instead.")
  (id m))

(cl:ensure-generic-function 'object_class-val :lambda-list '(m))
(cl:defmethod object_class-val ((m <FollowTargetInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mtt_msgs-msg:object_class-val is deprecated.  Use mtt_msgs-msg:object_class instead.")
  (object_class m))

(cl:ensure-generic-function 'score-val :lambda-list '(m))
(cl:defmethod score-val ((m <FollowTargetInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mtt_msgs-msg:score-val is deprecated.  Use mtt_msgs-msg:score instead.")
  (score m))

(cl:ensure-generic-function 'pose-val :lambda-list '(m))
(cl:defmethod pose-val ((m <FollowTargetInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mtt_msgs-msg:pose-val is deprecated.  Use mtt_msgs-msg:pose instead.")
  (pose m))

(cl:ensure-generic-function 'size-val :lambda-list '(m))
(cl:defmethod size-val ((m <FollowTargetInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mtt_msgs-msg:size-val is deprecated.  Use mtt_msgs-msg:size instead.")
  (size m))

(cl:ensure-generic-function 'velocity-val :lambda-list '(m))
(cl:defmethod velocity-val ((m <FollowTargetInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mtt_msgs-msg:velocity-val is deprecated.  Use mtt_msgs-msg:velocity instead.")
  (velocity m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FollowTargetInfo>) ostream)
  "Serializes a message object of type '<FollowTargetInfo>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'id) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'object_class) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'score) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'size) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'velocity) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FollowTargetInfo>) istream)
  "Deserializes a message object of type '<FollowTargetInfo>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'id) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'object_class) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'score) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'size) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'velocity) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FollowTargetInfo>)))
  "Returns string type for a message object of type '<FollowTargetInfo>"
  "mtt_msgs/FollowTargetInfo")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FollowTargetInfo)))
  "Returns string type for a message object of type 'FollowTargetInfo"
  "mtt_msgs/FollowTargetInfo")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FollowTargetInfo>)))
  "Returns md5sum for a message object of type '<FollowTargetInfo>"
  "f662c869093ee1a8404764360b3f4101")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FollowTargetInfo)))
  "Returns md5sum for a message object of type 'FollowTargetInfo"
  "f662c869093ee1a8404764360b3f4101")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FollowTargetInfo>)))
  "Returns full string definition for message of type '<FollowTargetInfo>"
  (cl:format cl:nil "std_msgs/Header header~%std_msgs/Int32 id~%std_msgs/Int8 object_class~%std_msgs/Float32 score~%geometry_msgs/Pose pose~%geometry_msgs/Point size~%geometry_msgs/Point velocity~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: std_msgs/Int32~%int32 data~%================================================================================~%MSG: std_msgs/Int8~%int8 data~%~%================================================================================~%MSG: std_msgs/Float32~%float32 data~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FollowTargetInfo)))
  "Returns full string definition for message of type 'FollowTargetInfo"
  (cl:format cl:nil "std_msgs/Header header~%std_msgs/Int32 id~%std_msgs/Int8 object_class~%std_msgs/Float32 score~%geometry_msgs/Pose pose~%geometry_msgs/Point size~%geometry_msgs/Point velocity~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: std_msgs/Int32~%int32 data~%================================================================================~%MSG: std_msgs/Int8~%int8 data~%~%================================================================================~%MSG: std_msgs/Float32~%float32 data~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FollowTargetInfo>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'id))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'object_class))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'score))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'size))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'velocity))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FollowTargetInfo>))
  "Converts a ROS message object to a list"
  (cl:list 'FollowTargetInfo
    (cl:cons ':header (header msg))
    (cl:cons ':id (id msg))
    (cl:cons ':object_class (object_class msg))
    (cl:cons ':score (score msg))
    (cl:cons ':pose (pose msg))
    (cl:cons ':size (size msg))
    (cl:cons ':velocity (velocity msg))
))
