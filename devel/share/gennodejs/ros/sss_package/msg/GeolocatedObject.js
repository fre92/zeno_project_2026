// Auto-generated. Do not edit!

// (in-package sss_package.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class GeolocatedObject {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.object_class = null;
      this.confidence = null;
      this.latitude = null;
      this.longitude = null;
      this.ping_index = null;
      this.ping_stamp = null;
      this.centroid_x_px = null;
      this.centroid_y_px = null;
      this.bbox_x_px = null;
      this.bbox_y_px = null;
      this.bbox_width_px = null;
      this.bbox_height_px = null;
    }
    else {
      if (initObj.hasOwnProperty('object_class')) {
        this.object_class = initObj.object_class
      }
      else {
        this.object_class = '';
      }
      if (initObj.hasOwnProperty('confidence')) {
        this.confidence = initObj.confidence
      }
      else {
        this.confidence = 0.0;
      }
      if (initObj.hasOwnProperty('latitude')) {
        this.latitude = initObj.latitude
      }
      else {
        this.latitude = 0.0;
      }
      if (initObj.hasOwnProperty('longitude')) {
        this.longitude = initObj.longitude
      }
      else {
        this.longitude = 0.0;
      }
      if (initObj.hasOwnProperty('ping_index')) {
        this.ping_index = initObj.ping_index
      }
      else {
        this.ping_index = 0;
      }
      if (initObj.hasOwnProperty('ping_stamp')) {
        this.ping_stamp = initObj.ping_stamp
      }
      else {
        this.ping_stamp = {secs: 0, nsecs: 0};
      }
      if (initObj.hasOwnProperty('centroid_x_px')) {
        this.centroid_x_px = initObj.centroid_x_px
      }
      else {
        this.centroid_x_px = 0.0;
      }
      if (initObj.hasOwnProperty('centroid_y_px')) {
        this.centroid_y_px = initObj.centroid_y_px
      }
      else {
        this.centroid_y_px = 0.0;
      }
      if (initObj.hasOwnProperty('bbox_x_px')) {
        this.bbox_x_px = initObj.bbox_x_px
      }
      else {
        this.bbox_x_px = 0;
      }
      if (initObj.hasOwnProperty('bbox_y_px')) {
        this.bbox_y_px = initObj.bbox_y_px
      }
      else {
        this.bbox_y_px = 0;
      }
      if (initObj.hasOwnProperty('bbox_width_px')) {
        this.bbox_width_px = initObj.bbox_width_px
      }
      else {
        this.bbox_width_px = 0;
      }
      if (initObj.hasOwnProperty('bbox_height_px')) {
        this.bbox_height_px = initObj.bbox_height_px
      }
      else {
        this.bbox_height_px = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GeolocatedObject
    // Serialize message field [object_class]
    bufferOffset = _serializer.string(obj.object_class, buffer, bufferOffset);
    // Serialize message field [confidence]
    bufferOffset = _serializer.float32(obj.confidence, buffer, bufferOffset);
    // Serialize message field [latitude]
    bufferOffset = _serializer.float64(obj.latitude, buffer, bufferOffset);
    // Serialize message field [longitude]
    bufferOffset = _serializer.float64(obj.longitude, buffer, bufferOffset);
    // Serialize message field [ping_index]
    bufferOffset = _serializer.uint32(obj.ping_index, buffer, bufferOffset);
    // Serialize message field [ping_stamp]
    bufferOffset = _serializer.time(obj.ping_stamp, buffer, bufferOffset);
    // Serialize message field [centroid_x_px]
    bufferOffset = _serializer.float32(obj.centroid_x_px, buffer, bufferOffset);
    // Serialize message field [centroid_y_px]
    bufferOffset = _serializer.float32(obj.centroid_y_px, buffer, bufferOffset);
    // Serialize message field [bbox_x_px]
    bufferOffset = _serializer.int32(obj.bbox_x_px, buffer, bufferOffset);
    // Serialize message field [bbox_y_px]
    bufferOffset = _serializer.int32(obj.bbox_y_px, buffer, bufferOffset);
    // Serialize message field [bbox_width_px]
    bufferOffset = _serializer.int32(obj.bbox_width_px, buffer, bufferOffset);
    // Serialize message field [bbox_height_px]
    bufferOffset = _serializer.int32(obj.bbox_height_px, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GeolocatedObject
    let len;
    let data = new GeolocatedObject(null);
    // Deserialize message field [object_class]
    data.object_class = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [confidence]
    data.confidence = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [latitude]
    data.latitude = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [longitude]
    data.longitude = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [ping_index]
    data.ping_index = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [ping_stamp]
    data.ping_stamp = _deserializer.time(buffer, bufferOffset);
    // Deserialize message field [centroid_x_px]
    data.centroid_x_px = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [centroid_y_px]
    data.centroid_y_px = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [bbox_x_px]
    data.bbox_x_px = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [bbox_y_px]
    data.bbox_y_px = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [bbox_width_px]
    data.bbox_width_px = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [bbox_height_px]
    data.bbox_height_px = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.object_class.length;
    return length + 60;
  }

  static datatype() {
    // Returns string type for a message object
    return 'sss_package/GeolocatedObject';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '09b78b232110d33d86e8f0f2984e6e72';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string object_class
    float32 confidence
    
    float64 latitude
    float64 longitude
    
    uint32 ping_index
    time ping_stamp
    float32 centroid_x_px
    float32 centroid_y_px
    
    int32 bbox_x_px
    int32 bbox_y_px
    int32 bbox_width_px
    int32 bbox_height_px
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GeolocatedObject(null);
    if (msg.object_class !== undefined) {
      resolved.object_class = msg.object_class;
    }
    else {
      resolved.object_class = ''
    }

    if (msg.confidence !== undefined) {
      resolved.confidence = msg.confidence;
    }
    else {
      resolved.confidence = 0.0
    }

    if (msg.latitude !== undefined) {
      resolved.latitude = msg.latitude;
    }
    else {
      resolved.latitude = 0.0
    }

    if (msg.longitude !== undefined) {
      resolved.longitude = msg.longitude;
    }
    else {
      resolved.longitude = 0.0
    }

    if (msg.ping_index !== undefined) {
      resolved.ping_index = msg.ping_index;
    }
    else {
      resolved.ping_index = 0
    }

    if (msg.ping_stamp !== undefined) {
      resolved.ping_stamp = msg.ping_stamp;
    }
    else {
      resolved.ping_stamp = {secs: 0, nsecs: 0}
    }

    if (msg.centroid_x_px !== undefined) {
      resolved.centroid_x_px = msg.centroid_x_px;
    }
    else {
      resolved.centroid_x_px = 0.0
    }

    if (msg.centroid_y_px !== undefined) {
      resolved.centroid_y_px = msg.centroid_y_px;
    }
    else {
      resolved.centroid_y_px = 0.0
    }

    if (msg.bbox_x_px !== undefined) {
      resolved.bbox_x_px = msg.bbox_x_px;
    }
    else {
      resolved.bbox_x_px = 0
    }

    if (msg.bbox_y_px !== undefined) {
      resolved.bbox_y_px = msg.bbox_y_px;
    }
    else {
      resolved.bbox_y_px = 0
    }

    if (msg.bbox_width_px !== undefined) {
      resolved.bbox_width_px = msg.bbox_width_px;
    }
    else {
      resolved.bbox_width_px = 0
    }

    if (msg.bbox_height_px !== undefined) {
      resolved.bbox_height_px = msg.bbox_height_px;
    }
    else {
      resolved.bbox_height_px = 0
    }

    return resolved;
    }
};

module.exports = GeolocatedObject;
