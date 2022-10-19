// Auto-generated. Do not edit!

// (in-package mymsg.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class neighborpos {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.xpos = null;
      this.ypos = null;
      this.time_stamp = null;
    }
    else {
      if (initObj.hasOwnProperty('xpos')) {
        this.xpos = initObj.xpos
      }
      else {
        this.xpos = [];
      }
      if (initObj.hasOwnProperty('ypos')) {
        this.ypos = initObj.ypos
      }
      else {
        this.ypos = [];
      }
      if (initObj.hasOwnProperty('time_stamp')) {
        this.time_stamp = initObj.time_stamp
      }
      else {
        this.time_stamp = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type neighborpos
    // Serialize message field [xpos]
    bufferOffset = _arraySerializer.float64(obj.xpos, buffer, bufferOffset, null);
    // Serialize message field [ypos]
    bufferOffset = _arraySerializer.float64(obj.ypos, buffer, bufferOffset, null);
    // Serialize message field [time_stamp]
    bufferOffset = _serializer.float64(obj.time_stamp, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type neighborpos
    let len;
    let data = new neighborpos(null);
    // Deserialize message field [xpos]
    data.xpos = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [ypos]
    data.ypos = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [time_stamp]
    data.time_stamp = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 8 * object.xpos.length;
    length += 8 * object.ypos.length;
    return length + 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'mymsg/neighborpos';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3f32e5464578c19ccbb2ebfabda27e48';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64[] xpos
    float64[] ypos
    float64 time_stamp
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new neighborpos(null);
    if (msg.xpos !== undefined) {
      resolved.xpos = msg.xpos;
    }
    else {
      resolved.xpos = []
    }

    if (msg.ypos !== undefined) {
      resolved.ypos = msg.ypos;
    }
    else {
      resolved.ypos = []
    }

    if (msg.time_stamp !== undefined) {
      resolved.time_stamp = msg.time_stamp;
    }
    else {
      resolved.time_stamp = 0.0
    }

    return resolved;
    }
};

module.exports = neighborpos;
