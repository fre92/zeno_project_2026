// Auto-generated. Do not edit!

// (in-package sss_package.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let marta_msgs = _finder('marta_msgs');
let sensor_msgs = _finder('sensor_msgs');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class ImageMetadata {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.image = null;
      this.ping_indices = null;
      this.ping_stamps = null;
      this.nav_statuses = null;
      this.nav_valid = null;
      this.altitudes = null;
      this.altitude_valid = null;
      this.object_classes = null;
      this.object_confidences = null;
      this.object_centroid_x_px = null;
      this.object_centroid_y_px = null;
      this.object_bbox_x_px = null;
      this.object_bbox_y_px = null;
      this.object_bbox_width_px = null;
      this.object_bbox_height_px = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('image')) {
        this.image = initObj.image
      }
      else {
        this.image = new sensor_msgs.msg.Image();
      }
      if (initObj.hasOwnProperty('ping_indices')) {
        this.ping_indices = initObj.ping_indices
      }
      else {
        this.ping_indices = [];
      }
      if (initObj.hasOwnProperty('ping_stamps')) {
        this.ping_stamps = initObj.ping_stamps
      }
      else {
        this.ping_stamps = [];
      }
      if (initObj.hasOwnProperty('nav_statuses')) {
        this.nav_statuses = initObj.nav_statuses
      }
      else {
        this.nav_statuses = [];
      }
      if (initObj.hasOwnProperty('nav_valid')) {
        this.nav_valid = initObj.nav_valid
      }
      else {
        this.nav_valid = [];
      }
      if (initObj.hasOwnProperty('altitudes')) {
        this.altitudes = initObj.altitudes
      }
      else {
        this.altitudes = [];
      }
      if (initObj.hasOwnProperty('altitude_valid')) {
        this.altitude_valid = initObj.altitude_valid
      }
      else {
        this.altitude_valid = [];
      }
      if (initObj.hasOwnProperty('object_classes')) {
        this.object_classes = initObj.object_classes
      }
      else {
        this.object_classes = [];
      }
      if (initObj.hasOwnProperty('object_confidences')) {
        this.object_confidences = initObj.object_confidences
      }
      else {
        this.object_confidences = [];
      }
      if (initObj.hasOwnProperty('object_centroid_x_px')) {
        this.object_centroid_x_px = initObj.object_centroid_x_px
      }
      else {
        this.object_centroid_x_px = [];
      }
      if (initObj.hasOwnProperty('object_centroid_y_px')) {
        this.object_centroid_y_px = initObj.object_centroid_y_px
      }
      else {
        this.object_centroid_y_px = [];
      }
      if (initObj.hasOwnProperty('object_bbox_x_px')) {
        this.object_bbox_x_px = initObj.object_bbox_x_px
      }
      else {
        this.object_bbox_x_px = [];
      }
      if (initObj.hasOwnProperty('object_bbox_y_px')) {
        this.object_bbox_y_px = initObj.object_bbox_y_px
      }
      else {
        this.object_bbox_y_px = [];
      }
      if (initObj.hasOwnProperty('object_bbox_width_px')) {
        this.object_bbox_width_px = initObj.object_bbox_width_px
      }
      else {
        this.object_bbox_width_px = [];
      }
      if (initObj.hasOwnProperty('object_bbox_height_px')) {
        this.object_bbox_height_px = initObj.object_bbox_height_px
      }
      else {
        this.object_bbox_height_px = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ImageMetadata
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [image]
    bufferOffset = sensor_msgs.msg.Image.serialize(obj.image, buffer, bufferOffset);
    // Serialize message field [ping_indices]
    bufferOffset = _arraySerializer.uint32(obj.ping_indices, buffer, bufferOffset, null);
    // Serialize message field [ping_stamps]
    bufferOffset = _arraySerializer.time(obj.ping_stamps, buffer, bufferOffset, null);
    // Serialize message field [nav_statuses]
    // Serialize the length for message field [nav_statuses]
    bufferOffset = _serializer.uint32(obj.nav_statuses.length, buffer, bufferOffset);
    obj.nav_statuses.forEach((val) => {
      bufferOffset = marta_msgs.msg.NavStatus.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [nav_valid]
    bufferOffset = _arraySerializer.bool(obj.nav_valid, buffer, bufferOffset, null);
    // Serialize message field [altitudes]
    bufferOffset = _arraySerializer.float32(obj.altitudes, buffer, bufferOffset, null);
    // Serialize message field [altitude_valid]
    bufferOffset = _arraySerializer.bool(obj.altitude_valid, buffer, bufferOffset, null);
    // Serialize message field [object_classes]
    bufferOffset = _arraySerializer.string(obj.object_classes, buffer, bufferOffset, null);
    // Serialize message field [object_confidences]
    bufferOffset = _arraySerializer.float32(obj.object_confidences, buffer, bufferOffset, null);
    // Serialize message field [object_centroid_x_px]
    bufferOffset = _arraySerializer.float32(obj.object_centroid_x_px, buffer, bufferOffset, null);
    // Serialize message field [object_centroid_y_px]
    bufferOffset = _arraySerializer.float32(obj.object_centroid_y_px, buffer, bufferOffset, null);
    // Serialize message field [object_bbox_x_px]
    bufferOffset = _arraySerializer.int32(obj.object_bbox_x_px, buffer, bufferOffset, null);
    // Serialize message field [object_bbox_y_px]
    bufferOffset = _arraySerializer.int32(obj.object_bbox_y_px, buffer, bufferOffset, null);
    // Serialize message field [object_bbox_width_px]
    bufferOffset = _arraySerializer.int32(obj.object_bbox_width_px, buffer, bufferOffset, null);
    // Serialize message field [object_bbox_height_px]
    bufferOffset = _arraySerializer.int32(obj.object_bbox_height_px, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ImageMetadata
    let len;
    let data = new ImageMetadata(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [image]
    data.image = sensor_msgs.msg.Image.deserialize(buffer, bufferOffset);
    // Deserialize message field [ping_indices]
    data.ping_indices = _arrayDeserializer.uint32(buffer, bufferOffset, null)
    // Deserialize message field [ping_stamps]
    data.ping_stamps = _arrayDeserializer.time(buffer, bufferOffset, null)
    // Deserialize message field [nav_statuses]
    // Deserialize array length for message field [nav_statuses]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.nav_statuses = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.nav_statuses[i] = marta_msgs.msg.NavStatus.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [nav_valid]
    data.nav_valid = _arrayDeserializer.bool(buffer, bufferOffset, null)
    // Deserialize message field [altitudes]
    data.altitudes = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [altitude_valid]
    data.altitude_valid = _arrayDeserializer.bool(buffer, bufferOffset, null)
    // Deserialize message field [object_classes]
    data.object_classes = _arrayDeserializer.string(buffer, bufferOffset, null)
    // Deserialize message field [object_confidences]
    data.object_confidences = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [object_centroid_x_px]
    data.object_centroid_x_px = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [object_centroid_y_px]
    data.object_centroid_y_px = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [object_bbox_x_px]
    data.object_bbox_x_px = _arrayDeserializer.int32(buffer, bufferOffset, null)
    // Deserialize message field [object_bbox_y_px]
    data.object_bbox_y_px = _arrayDeserializer.int32(buffer, bufferOffset, null)
    // Deserialize message field [object_bbox_width_px]
    data.object_bbox_width_px = _arrayDeserializer.int32(buffer, bufferOffset, null)
    // Deserialize message field [object_bbox_height_px]
    data.object_bbox_height_px = _arrayDeserializer.int32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += sensor_msgs.msg.Image.getMessageSize(object.image);
    length += 4 * object.ping_indices.length;
    length += 8 * object.ping_stamps.length;
    object.nav_statuses.forEach((val) => {
      length += marta_msgs.msg.NavStatus.getMessageSize(val);
    });
    length += object.nav_valid.length;
    length += 4 * object.altitudes.length;
    length += object.altitude_valid.length;
    object.object_classes.forEach((val) => {
      length += 4 + val.length;
    });
    length += 4 * object.object_confidences.length;
    length += 4 * object.object_centroid_x_px.length;
    length += 4 * object.object_centroid_y_px.length;
    length += 4 * object.object_bbox_x_px.length;
    length += 4 * object.object_bbox_y_px.length;
    length += 4 * object.object_bbox_width_px.length;
    length += 4 * object.object_bbox_height_px.length;
    return length + 56;
  }

  static datatype() {
    // Returns string type for a message object
    return 'sss_package/ImageMetadata';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b7d71bb022c02e7c92c518f300d617c9';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    
    sensor_msgs/Image image
    
    uint32[] ping_indices
    time[] ping_stamps
    marta_msgs/NavStatus[] nav_statuses
    bool[] nav_valid
    float32[] altitudes
    bool[] altitude_valid
    
    string[] object_classes
    float32[] object_confidences
    float32[] object_centroid_x_px
    float32[] object_centroid_y_px
    int32[] object_bbox_x_px
    int32[] object_bbox_y_px
    int32[] object_bbox_width_px
    int32[] object_bbox_height_px
    
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
    MSG: sensor_msgs/Image
    # This message contains an uncompressed image
    # (0, 0) is at top-left corner of image
    #
    
    Header header        # Header timestamp should be acquisition time of image
                         # Header frame_id should be optical frame of camera
                         # origin of frame should be optical center of camera
                         # +x should point to the right in the image
                         # +y should point down in the image
                         # +z should point into to plane of the image
                         # If the frame_id here and the frame_id of the CameraInfo
                         # message associated with the image conflict
                         # the behavior is undefined
    
    uint32 height         # image height, that is, number of rows
    uint32 width          # image width, that is, number of columns
    
    # The legal values for encoding are in file src/image_encodings.cpp
    # If you want to standardize a new string format, join
    # ros-users@lists.sourceforge.net and send an email proposing a new encoding.
    
    string encoding       # Encoding of pixels -- channel meaning, ordering, size
                          # taken from the list of strings in include/sensor_msgs/image_encodings.h
    
    uint8 is_bigendian    # is this data bigendian?
    uint32 step           # Full row length in bytes
    uint8[] data          # actual matrix data, size is (step * rows)
    
    ================================================================================
    MSG: marta_msgs/NavStatus
    Header header
    
    marta_msgs/Position position
     
    marta_msgs/Euler orientation
    
    marta_msgs/Quaternion quaternion
    
    geometry_msgs/Vector3 ned_speed
    
    geometry_msgs/Vector3 omega_body
    
    uint8 gps_status
    
    bool initialized
    
    
    ================================================================================
    MSG: marta_msgs/Position
    float64 latitude
    float64 longitude
    float64 depth
    
    ================================================================================
    MSG: marta_msgs/Euler
    float64 roll
    float64 pitch
    float64 yaw
    
    ================================================================================
    MSG: marta_msgs/Quaternion
    float64 w
    float64 x
    float64 y
    float64 z
    
    ================================================================================
    MSG: geometry_msgs/Vector3
    # This represents a vector in free space. 
    # It is only meant to represent a direction. Therefore, it does not
    # make sense to apply a translation to it (e.g., when applying a 
    # generic rigid transformation to a Vector3, tf2 will only apply the
    # rotation). If you want your data to be translatable too, use the
    # geometry_msgs/Point message instead.
    
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
    const resolved = new ImageMetadata(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.image !== undefined) {
      resolved.image = sensor_msgs.msg.Image.Resolve(msg.image)
    }
    else {
      resolved.image = new sensor_msgs.msg.Image()
    }

    if (msg.ping_indices !== undefined) {
      resolved.ping_indices = msg.ping_indices;
    }
    else {
      resolved.ping_indices = []
    }

    if (msg.ping_stamps !== undefined) {
      resolved.ping_stamps = msg.ping_stamps;
    }
    else {
      resolved.ping_stamps = []
    }

    if (msg.nav_statuses !== undefined) {
      resolved.nav_statuses = new Array(msg.nav_statuses.length);
      for (let i = 0; i < resolved.nav_statuses.length; ++i) {
        resolved.nav_statuses[i] = marta_msgs.msg.NavStatus.Resolve(msg.nav_statuses[i]);
      }
    }
    else {
      resolved.nav_statuses = []
    }

    if (msg.nav_valid !== undefined) {
      resolved.nav_valid = msg.nav_valid;
    }
    else {
      resolved.nav_valid = []
    }

    if (msg.altitudes !== undefined) {
      resolved.altitudes = msg.altitudes;
    }
    else {
      resolved.altitudes = []
    }

    if (msg.altitude_valid !== undefined) {
      resolved.altitude_valid = msg.altitude_valid;
    }
    else {
      resolved.altitude_valid = []
    }

    if (msg.object_classes !== undefined) {
      resolved.object_classes = msg.object_classes;
    }
    else {
      resolved.object_classes = []
    }

    if (msg.object_confidences !== undefined) {
      resolved.object_confidences = msg.object_confidences;
    }
    else {
      resolved.object_confidences = []
    }

    if (msg.object_centroid_x_px !== undefined) {
      resolved.object_centroid_x_px = msg.object_centroid_x_px;
    }
    else {
      resolved.object_centroid_x_px = []
    }

    if (msg.object_centroid_y_px !== undefined) {
      resolved.object_centroid_y_px = msg.object_centroid_y_px;
    }
    else {
      resolved.object_centroid_y_px = []
    }

    if (msg.object_bbox_x_px !== undefined) {
      resolved.object_bbox_x_px = msg.object_bbox_x_px;
    }
    else {
      resolved.object_bbox_x_px = []
    }

    if (msg.object_bbox_y_px !== undefined) {
      resolved.object_bbox_y_px = msg.object_bbox_y_px;
    }
    else {
      resolved.object_bbox_y_px = []
    }

    if (msg.object_bbox_width_px !== undefined) {
      resolved.object_bbox_width_px = msg.object_bbox_width_px;
    }
    else {
      resolved.object_bbox_width_px = []
    }

    if (msg.object_bbox_height_px !== undefined) {
      resolved.object_bbox_height_px = msg.object_bbox_height_px;
    }
    else {
      resolved.object_bbox_height_px = []
    }

    return resolved;
    }
};

module.exports = ImageMetadata;
