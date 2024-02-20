# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from mtt_msgs/FollowTargetInfo.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import geometry_msgs.msg
import std_msgs.msg

class FollowTargetInfo(genpy.Message):
  _md5sum = "f662c869093ee1a8404764360b3f4101"
  _type = "mtt_msgs/FollowTargetInfo"
  _has_header = True  # flag to mark the presence of a Header object
  _full_text = """std_msgs/Header header
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
"""
  __slots__ = ['header','id','object_class','score','pose','size','velocity']
  _slot_types = ['std_msgs/Header','std_msgs/Int32','std_msgs/Int8','std_msgs/Float32','geometry_msgs/Pose','geometry_msgs/Point','geometry_msgs/Point']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header,id,object_class,score,pose,size,velocity

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(FollowTargetInfo, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.id is None:
        self.id = std_msgs.msg.Int32()
      if self.object_class is None:
        self.object_class = std_msgs.msg.Int8()
      if self.score is None:
        self.score = std_msgs.msg.Float32()
      if self.pose is None:
        self.pose = geometry_msgs.msg.Pose()
      if self.size is None:
        self.size = geometry_msgs.msg.Point()
      if self.velocity is None:
        self.velocity = geometry_msgs.msg.Point()
    else:
      self.header = std_msgs.msg.Header()
      self.id = std_msgs.msg.Int32()
      self.object_class = std_msgs.msg.Int8()
      self.score = std_msgs.msg.Float32()
      self.pose = geometry_msgs.msg.Pose()
      self.size = geometry_msgs.msg.Point()
      self.velocity = geometry_msgs.msg.Point()

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_get_struct_3I().pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_ibf13d().pack(_x.id.data, _x.object_class.data, _x.score.data, _x.pose.position.x, _x.pose.position.y, _x.pose.position.z, _x.pose.orientation.x, _x.pose.orientation.y, _x.pose.orientation.z, _x.pose.orientation.w, _x.size.x, _x.size.y, _x.size.z, _x.velocity.x, _x.velocity.y, _x.velocity.z))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.id is None:
        self.id = std_msgs.msg.Int32()
      if self.object_class is None:
        self.object_class = std_msgs.msg.Int8()
      if self.score is None:
        self.score = std_msgs.msg.Float32()
      if self.pose is None:
        self.pose = geometry_msgs.msg.Pose()
      if self.size is None:
        self.size = geometry_msgs.msg.Point()
      if self.velocity is None:
        self.velocity = geometry_msgs.msg.Point()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 113
      (_x.id.data, _x.object_class.data, _x.score.data, _x.pose.position.x, _x.pose.position.y, _x.pose.position.z, _x.pose.orientation.x, _x.pose.orientation.y, _x.pose.orientation.z, _x.pose.orientation.w, _x.size.x, _x.size.y, _x.size.z, _x.velocity.x, _x.velocity.y, _x.velocity.z,) = _get_struct_ibf13d().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_3I().pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_ibf13d().pack(_x.id.data, _x.object_class.data, _x.score.data, _x.pose.position.x, _x.pose.position.y, _x.pose.position.z, _x.pose.orientation.x, _x.pose.orientation.y, _x.pose.orientation.z, _x.pose.orientation.w, _x.size.x, _x.size.y, _x.size.z, _x.velocity.x, _x.velocity.y, _x.velocity.z))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.id is None:
        self.id = std_msgs.msg.Int32()
      if self.object_class is None:
        self.object_class = std_msgs.msg.Int8()
      if self.score is None:
        self.score = std_msgs.msg.Float32()
      if self.pose is None:
        self.pose = geometry_msgs.msg.Pose()
      if self.size is None:
        self.size = geometry_msgs.msg.Point()
      if self.velocity is None:
        self.velocity = geometry_msgs.msg.Point()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 113
      (_x.id.data, _x.object_class.data, _x.score.data, _x.pose.position.x, _x.pose.position.y, _x.pose.position.z, _x.pose.orientation.x, _x.pose.orientation.y, _x.pose.orientation.z, _x.pose.orientation.w, _x.size.x, _x.size.y, _x.size.z, _x.velocity.x, _x.velocity.y, _x.velocity.z,) = _get_struct_ibf13d().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_3I = None
def _get_struct_3I():
    global _struct_3I
    if _struct_3I is None:
        _struct_3I = struct.Struct("<3I")
    return _struct_3I
_struct_ibf13d = None
def _get_struct_ibf13d():
    global _struct_ibf13d
    if _struct_ibf13d is None:
        _struct_ibf13d = struct.Struct("<ibf13d")
    return _struct_ibf13d
