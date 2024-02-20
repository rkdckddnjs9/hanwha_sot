; Auto-generated. Do not edit!


(cl:in-package mtt_msgs-msg)


;//! \htmlinclude TargetCandidate.msg.html

(cl:defclass <TargetCandidate> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (id_1
    :reader id_1
    :initarg :id_1
    :type std_msgs-msg:Int32
    :initform (cl:make-instance 'std_msgs-msg:Int32))
   (class_1
    :reader class_1
    :initarg :class_1
    :type std_msgs-msg:Int8
    :initform (cl:make-instance 'std_msgs-msg:Int8))
   (position_1
    :reader position_1
    :initarg :position_1
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (id_2
    :reader id_2
    :initarg :id_2
    :type std_msgs-msg:Int32
    :initform (cl:make-instance 'std_msgs-msg:Int32))
   (class_2
    :reader class_2
    :initarg :class_2
    :type std_msgs-msg:Int8
    :initform (cl:make-instance 'std_msgs-msg:Int8))
   (position_2
    :reader position_2
    :initarg :position_2
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (id_3
    :reader id_3
    :initarg :id_3
    :type std_msgs-msg:Int32
    :initform (cl:make-instance 'std_msgs-msg:Int32))
   (class_3
    :reader class_3
    :initarg :class_3
    :type std_msgs-msg:Int8
    :initform (cl:make-instance 'std_msgs-msg:Int8))
   (position_3
    :reader position_3
    :initarg :position_3
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (id_4
    :reader id_4
    :initarg :id_4
    :type std_msgs-msg:Int32
    :initform (cl:make-instance 'std_msgs-msg:Int32))
   (class_4
    :reader class_4
    :initarg :class_4
    :type std_msgs-msg:Int8
    :initform (cl:make-instance 'std_msgs-msg:Int8))
   (position_4
    :reader position_4
    :initarg :position_4
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (id_5
    :reader id_5
    :initarg :id_5
    :type std_msgs-msg:Int32
    :initform (cl:make-instance 'std_msgs-msg:Int32))
   (class_5
    :reader class_5
    :initarg :class_5
    :type std_msgs-msg:Int8
    :initform (cl:make-instance 'std_msgs-msg:Int8))
   (position_5
    :reader position_5
    :initarg :position_5
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point)))
)

(cl:defclass TargetCandidate (<TargetCandidate>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TargetCandidate>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TargetCandidate)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mtt_msgs-msg:<TargetCandidate> is deprecated: use mtt_msgs-msg:TargetCandidate instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <TargetCandidate>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mtt_msgs-msg:header-val is deprecated.  Use mtt_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'id_1-val :lambda-list '(m))
(cl:defmethod id_1-val ((m <TargetCandidate>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mtt_msgs-msg:id_1-val is deprecated.  Use mtt_msgs-msg:id_1 instead.")
  (id_1 m))

(cl:ensure-generic-function 'class_1-val :lambda-list '(m))
(cl:defmethod class_1-val ((m <TargetCandidate>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mtt_msgs-msg:class_1-val is deprecated.  Use mtt_msgs-msg:class_1 instead.")
  (class_1 m))

(cl:ensure-generic-function 'position_1-val :lambda-list '(m))
(cl:defmethod position_1-val ((m <TargetCandidate>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mtt_msgs-msg:position_1-val is deprecated.  Use mtt_msgs-msg:position_1 instead.")
  (position_1 m))

(cl:ensure-generic-function 'id_2-val :lambda-list '(m))
(cl:defmethod id_2-val ((m <TargetCandidate>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mtt_msgs-msg:id_2-val is deprecated.  Use mtt_msgs-msg:id_2 instead.")
  (id_2 m))

(cl:ensure-generic-function 'class_2-val :lambda-list '(m))
(cl:defmethod class_2-val ((m <TargetCandidate>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mtt_msgs-msg:class_2-val is deprecated.  Use mtt_msgs-msg:class_2 instead.")
  (class_2 m))

(cl:ensure-generic-function 'position_2-val :lambda-list '(m))
(cl:defmethod position_2-val ((m <TargetCandidate>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mtt_msgs-msg:position_2-val is deprecated.  Use mtt_msgs-msg:position_2 instead.")
  (position_2 m))

(cl:ensure-generic-function 'id_3-val :lambda-list '(m))
(cl:defmethod id_3-val ((m <TargetCandidate>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mtt_msgs-msg:id_3-val is deprecated.  Use mtt_msgs-msg:id_3 instead.")
  (id_3 m))

(cl:ensure-generic-function 'class_3-val :lambda-list '(m))
(cl:defmethod class_3-val ((m <TargetCandidate>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mtt_msgs-msg:class_3-val is deprecated.  Use mtt_msgs-msg:class_3 instead.")
  (class_3 m))

(cl:ensure-generic-function 'position_3-val :lambda-list '(m))
(cl:defmethod position_3-val ((m <TargetCandidate>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mtt_msgs-msg:position_3-val is deprecated.  Use mtt_msgs-msg:position_3 instead.")
  (position_3 m))

(cl:ensure-generic-function 'id_4-val :lambda-list '(m))
(cl:defmethod id_4-val ((m <TargetCandidate>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mtt_msgs-msg:id_4-val is deprecated.  Use mtt_msgs-msg:id_4 instead.")
  (id_4 m))

(cl:ensure-generic-function 'class_4-val :lambda-list '(m))
(cl:defmethod class_4-val ((m <TargetCandidate>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mtt_msgs-msg:class_4-val is deprecated.  Use mtt_msgs-msg:class_4 instead.")
  (class_4 m))

(cl:ensure-generic-function 'position_4-val :lambda-list '(m))
(cl:defmethod position_4-val ((m <TargetCandidate>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mtt_msgs-msg:position_4-val is deprecated.  Use mtt_msgs-msg:position_4 instead.")
  (position_4 m))

(cl:ensure-generic-function 'id_5-val :lambda-list '(m))
(cl:defmethod id_5-val ((m <TargetCandidate>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mtt_msgs-msg:id_5-val is deprecated.  Use mtt_msgs-msg:id_5 instead.")
  (id_5 m))

(cl:ensure-generic-function 'class_5-val :lambda-list '(m))
(cl:defmethod class_5-val ((m <TargetCandidate>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mtt_msgs-msg:class_5-val is deprecated.  Use mtt_msgs-msg:class_5 instead.")
  (class_5 m))

(cl:ensure-generic-function 'position_5-val :lambda-list '(m))
(cl:defmethod position_5-val ((m <TargetCandidate>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mtt_msgs-msg:position_5-val is deprecated.  Use mtt_msgs-msg:position_5 instead.")
  (position_5 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TargetCandidate>) ostream)
  "Serializes a message object of type '<TargetCandidate>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'id_1) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'class_1) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'position_1) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'id_2) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'class_2) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'position_2) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'id_3) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'class_3) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'position_3) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'id_4) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'class_4) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'position_4) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'id_5) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'class_5) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'position_5) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TargetCandidate>) istream)
  "Deserializes a message object of type '<TargetCandidate>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'id_1) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'class_1) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'position_1) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'id_2) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'class_2) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'position_2) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'id_3) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'class_3) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'position_3) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'id_4) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'class_4) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'position_4) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'id_5) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'class_5) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'position_5) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TargetCandidate>)))
  "Returns string type for a message object of type '<TargetCandidate>"
  "mtt_msgs/TargetCandidate")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TargetCandidate)))
  "Returns string type for a message object of type 'TargetCandidate"
  "mtt_msgs/TargetCandidate")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TargetCandidate>)))
  "Returns md5sum for a message object of type '<TargetCandidate>"
  "fc377ddc3099b4c637aba5f3b76231f6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TargetCandidate)))
  "Returns md5sum for a message object of type 'TargetCandidate"
  "fc377ddc3099b4c637aba5f3b76231f6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TargetCandidate>)))
  "Returns full string definition for message of type '<TargetCandidate>"
  (cl:format cl:nil "std_msgs/Header header~%std_msgs/Int32 id_1~%std_msgs/Int8 class_1~%geometry_msgs/Point position_1~%std_msgs/Int32 id_2~%std_msgs/Int8 class_2~%geometry_msgs/Point position_2~%std_msgs/Int32 id_3~%std_msgs/Int8 class_3~%geometry_msgs/Point position_3~%std_msgs/Int32 id_4~%std_msgs/Int8 class_4~%geometry_msgs/Point position_4~%std_msgs/Int32 id_5~%std_msgs/Int8 class_5~%geometry_msgs/Point position_5~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: std_msgs/Int32~%int32 data~%================================================================================~%MSG: std_msgs/Int8~%int8 data~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TargetCandidate)))
  "Returns full string definition for message of type 'TargetCandidate"
  (cl:format cl:nil "std_msgs/Header header~%std_msgs/Int32 id_1~%std_msgs/Int8 class_1~%geometry_msgs/Point position_1~%std_msgs/Int32 id_2~%std_msgs/Int8 class_2~%geometry_msgs/Point position_2~%std_msgs/Int32 id_3~%std_msgs/Int8 class_3~%geometry_msgs/Point position_3~%std_msgs/Int32 id_4~%std_msgs/Int8 class_4~%geometry_msgs/Point position_4~%std_msgs/Int32 id_5~%std_msgs/Int8 class_5~%geometry_msgs/Point position_5~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: std_msgs/Int32~%int32 data~%================================================================================~%MSG: std_msgs/Int8~%int8 data~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TargetCandidate>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'id_1))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'class_1))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'position_1))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'id_2))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'class_2))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'position_2))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'id_3))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'class_3))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'position_3))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'id_4))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'class_4))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'position_4))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'id_5))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'class_5))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'position_5))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TargetCandidate>))
  "Converts a ROS message object to a list"
  (cl:list 'TargetCandidate
    (cl:cons ':header (header msg))
    (cl:cons ':id_1 (id_1 msg))
    (cl:cons ':class_1 (class_1 msg))
    (cl:cons ':position_1 (position_1 msg))
    (cl:cons ':id_2 (id_2 msg))
    (cl:cons ':class_2 (class_2 msg))
    (cl:cons ':position_2 (position_2 msg))
    (cl:cons ':id_3 (id_3 msg))
    (cl:cons ':class_3 (class_3 msg))
    (cl:cons ':position_3 (position_3 msg))
    (cl:cons ':id_4 (id_4 msg))
    (cl:cons ':class_4 (class_4 msg))
    (cl:cons ':position_4 (position_4 msg))
    (cl:cons ':id_5 (id_5 msg))
    (cl:cons ':class_5 (class_5 msg))
    (cl:cons ':position_5 (position_5 msg))
))
