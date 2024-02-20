// Auto-generated. Do not edit!

// (in-package mtt_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class TargetCandidate {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.id_1 = null;
      this.class_1 = null;
      this.position_1 = null;
      this.id_2 = null;
      this.class_2 = null;
      this.position_2 = null;
      this.id_3 = null;
      this.class_3 = null;
      this.position_3 = null;
      this.id_4 = null;
      this.class_4 = null;
      this.position_4 = null;
      this.id_5 = null;
      this.class_5 = null;
      this.position_5 = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('id_1')) {
        this.id_1 = initObj.id_1
      }
      else {
        this.id_1 = new std_msgs.msg.Int32();
      }
      if (initObj.hasOwnProperty('class_1')) {
        this.class_1 = initObj.class_1
      }
      else {
        this.class_1 = new std_msgs.msg.Int8();
      }
      if (initObj.hasOwnProperty('position_1')) {
        this.position_1 = initObj.position_1
      }
      else {
        this.position_1 = new geometry_msgs.msg.Point();
      }
      if (initObj.hasOwnProperty('id_2')) {
        this.id_2 = initObj.id_2
      }
      else {
        this.id_2 = new std_msgs.msg.Int32();
      }
      if (initObj.hasOwnProperty('class_2')) {
        this.class_2 = initObj.class_2
      }
      else {
        this.class_2 = new std_msgs.msg.Int8();
      }
      if (initObj.hasOwnProperty('position_2')) {
        this.position_2 = initObj.position_2
      }
      else {
        this.position_2 = new geometry_msgs.msg.Point();
      }
      if (initObj.hasOwnProperty('id_3')) {
        this.id_3 = initObj.id_3
      }
      else {
        this.id_3 = new std_msgs.msg.Int32();
      }
      if (initObj.hasOwnProperty('class_3')) {
        this.class_3 = initObj.class_3
      }
      else {
        this.class_3 = new std_msgs.msg.Int8();
      }
      if (initObj.hasOwnProperty('position_3')) {
        this.position_3 = initObj.position_3
      }
      else {
        this.position_3 = new geometry_msgs.msg.Point();
      }
      if (initObj.hasOwnProperty('id_4')) {
        this.id_4 = initObj.id_4
      }
      else {
        this.id_4 = new std_msgs.msg.Int32();
      }
      if (initObj.hasOwnProperty('class_4')) {
        this.class_4 = initObj.class_4
      }
      else {
        this.class_4 = new std_msgs.msg.Int8();
      }
      if (initObj.hasOwnProperty('position_4')) {
        this.position_4 = initObj.position_4
      }
      else {
        this.position_4 = new geometry_msgs.msg.Point();
      }
      if (initObj.hasOwnProperty('id_5')) {
        this.id_5 = initObj.id_5
      }
      else {
        this.id_5 = new std_msgs.msg.Int32();
      }
      if (initObj.hasOwnProperty('class_5')) {
        this.class_5 = initObj.class_5
      }
      else {
        this.class_5 = new std_msgs.msg.Int8();
      }
      if (initObj.hasOwnProperty('position_5')) {
        this.position_5 = initObj.position_5
      }
      else {
        this.position_5 = new geometry_msgs.msg.Point();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TargetCandidate
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [id_1]
    bufferOffset = std_msgs.msg.Int32.serialize(obj.id_1, buffer, bufferOffset);
    // Serialize message field [class_1]
    bufferOffset = std_msgs.msg.Int8.serialize(obj.class_1, buffer, bufferOffset);
    // Serialize message field [position_1]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.position_1, buffer, bufferOffset);
    // Serialize message field [id_2]
    bufferOffset = std_msgs.msg.Int32.serialize(obj.id_2, buffer, bufferOffset);
    // Serialize message field [class_2]
    bufferOffset = std_msgs.msg.Int8.serialize(obj.class_2, buffer, bufferOffset);
    // Serialize message field [position_2]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.position_2, buffer, bufferOffset);
    // Serialize message field [id_3]
    bufferOffset = std_msgs.msg.Int32.serialize(obj.id_3, buffer, bufferOffset);
    // Serialize message field [class_3]
    bufferOffset = std_msgs.msg.Int8.serialize(obj.class_3, buffer, bufferOffset);
    // Serialize message field [position_3]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.position_3, buffer, bufferOffset);
    // Serialize message field [id_4]
    bufferOffset = std_msgs.msg.Int32.serialize(obj.id_4, buffer, bufferOffset);
    // Serialize message field [class_4]
    bufferOffset = std_msgs.msg.Int8.serialize(obj.class_4, buffer, bufferOffset);
    // Serialize message field [position_4]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.position_4, buffer, bufferOffset);
    // Serialize message field [id_5]
    bufferOffset = std_msgs.msg.Int32.serialize(obj.id_5, buffer, bufferOffset);
    // Serialize message field [class_5]
    bufferOffset = std_msgs.msg.Int8.serialize(obj.class_5, buffer, bufferOffset);
    // Serialize message field [position_5]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.position_5, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TargetCandidate
    let len;
    let data = new TargetCandidate(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [id_1]
    data.id_1 = std_msgs.msg.Int32.deserialize(buffer, bufferOffset);
    // Deserialize message field [class_1]
    data.class_1 = std_msgs.msg.Int8.deserialize(buffer, bufferOffset);
    // Deserialize message field [position_1]
    data.position_1 = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    // Deserialize message field [id_2]
    data.id_2 = std_msgs.msg.Int32.deserialize(buffer, bufferOffset);
    // Deserialize message field [class_2]
    data.class_2 = std_msgs.msg.Int8.deserialize(buffer, bufferOffset);
    // Deserialize message field [position_2]
    data.position_2 = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    // Deserialize message field [id_3]
    data.id_3 = std_msgs.msg.Int32.deserialize(buffer, bufferOffset);
    // Deserialize message field [class_3]
    data.class_3 = std_msgs.msg.Int8.deserialize(buffer, bufferOffset);
    // Deserialize message field [position_3]
    data.position_3 = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    // Deserialize message field [id_4]
    data.id_4 = std_msgs.msg.Int32.deserialize(buffer, bufferOffset);
    // Deserialize message field [class_4]
    data.class_4 = std_msgs.msg.Int8.deserialize(buffer, bufferOffset);
    // Deserialize message field [position_4]
    data.position_4 = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    // Deserialize message field [id_5]
    data.id_5 = std_msgs.msg.Int32.deserialize(buffer, bufferOffset);
    // Deserialize message field [class_5]
    data.class_5 = std_msgs.msg.Int8.deserialize(buffer, bufferOffset);
    // Deserialize message field [position_5]
    data.position_5 = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 145;
  }

  static datatype() {
    // Returns string type for a message object
    return 'mtt_msgs/TargetCandidate';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'fc377ddc3099b4c637aba5f3b76231f6';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new TargetCandidate(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.id_1 !== undefined) {
      resolved.id_1 = std_msgs.msg.Int32.Resolve(msg.id_1)
    }
    else {
      resolved.id_1 = new std_msgs.msg.Int32()
    }

    if (msg.class_1 !== undefined) {
      resolved.class_1 = std_msgs.msg.Int8.Resolve(msg.class_1)
    }
    else {
      resolved.class_1 = new std_msgs.msg.Int8()
    }

    if (msg.position_1 !== undefined) {
      resolved.position_1 = geometry_msgs.msg.Point.Resolve(msg.position_1)
    }
    else {
      resolved.position_1 = new geometry_msgs.msg.Point()
    }

    if (msg.id_2 !== undefined) {
      resolved.id_2 = std_msgs.msg.Int32.Resolve(msg.id_2)
    }
    else {
      resolved.id_2 = new std_msgs.msg.Int32()
    }

    if (msg.class_2 !== undefined) {
      resolved.class_2 = std_msgs.msg.Int8.Resolve(msg.class_2)
    }
    else {
      resolved.class_2 = new std_msgs.msg.Int8()
    }

    if (msg.position_2 !== undefined) {
      resolved.position_2 = geometry_msgs.msg.Point.Resolve(msg.position_2)
    }
    else {
      resolved.position_2 = new geometry_msgs.msg.Point()
    }

    if (msg.id_3 !== undefined) {
      resolved.id_3 = std_msgs.msg.Int32.Resolve(msg.id_3)
    }
    else {
      resolved.id_3 = new std_msgs.msg.Int32()
    }

    if (msg.class_3 !== undefined) {
      resolved.class_3 = std_msgs.msg.Int8.Resolve(msg.class_3)
    }
    else {
      resolved.class_3 = new std_msgs.msg.Int8()
    }

    if (msg.position_3 !== undefined) {
      resolved.position_3 = geometry_msgs.msg.Point.Resolve(msg.position_3)
    }
    else {
      resolved.position_3 = new geometry_msgs.msg.Point()
    }

    if (msg.id_4 !== undefined) {
      resolved.id_4 = std_msgs.msg.Int32.Resolve(msg.id_4)
    }
    else {
      resolved.id_4 = new std_msgs.msg.Int32()
    }

    if (msg.class_4 !== undefined) {
      resolved.class_4 = std_msgs.msg.Int8.Resolve(msg.class_4)
    }
    else {
      resolved.class_4 = new std_msgs.msg.Int8()
    }

    if (msg.position_4 !== undefined) {
      resolved.position_4 = geometry_msgs.msg.Point.Resolve(msg.position_4)
    }
    else {
      resolved.position_4 = new geometry_msgs.msg.Point()
    }

    if (msg.id_5 !== undefined) {
      resolved.id_5 = std_msgs.msg.Int32.Resolve(msg.id_5)
    }
    else {
      resolved.id_5 = new std_msgs.msg.Int32()
    }

    if (msg.class_5 !== undefined) {
      resolved.class_5 = std_msgs.msg.Int8.Resolve(msg.class_5)
    }
    else {
      resolved.class_5 = new std_msgs.msg.Int8()
    }

    if (msg.position_5 !== undefined) {
      resolved.position_5 = geometry_msgs.msg.Point.Resolve(msg.position_5)
    }
    else {
      resolved.position_5 = new geometry_msgs.msg.Point()
    }

    return resolved;
    }
};

module.exports = TargetCandidate;
