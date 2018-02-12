// Auto-generated. Do not edit!

// (in-package mp_mini_picker.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class moveToPoseTcpRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.pose = null;
    }
    else {
      if (initObj.hasOwnProperty('pose')) {
        this.pose = initObj.pose
      }
      else {
        this.pose = new Array(6).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type moveToPoseTcpRequest
    // Check that the constant length array field [pose] has the right length
    if (obj.pose.length !== 6) {
      throw new Error('Unable to serialize array field pose - length must be 6')
    }
    // Serialize message field [pose]
    bufferOffset = _arraySerializer.float64(obj.pose, buffer, bufferOffset, 6);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type moveToPoseTcpRequest
    let len;
    let data = new moveToPoseTcpRequest(null);
    // Deserialize message field [pose]
    data.pose = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    return data;
  }

  static getMessageSize(object) {
    return 48;
  }

  static datatype() {
    // Returns string type for a service object
    return 'mp_mini_picker/moveToPoseTcpRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b7041cec88efffb038375323dab36ac5';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64[6] pose
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new moveToPoseTcpRequest(null);
    if (msg.pose !== undefined) {
      resolved.pose = msg.pose;
    }
    else {
      resolved.pose = new Array(6).fill(0)
    }

    return resolved;
    }
};

class moveToPoseTcpResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.ok = null;
    }
    else {
      if (initObj.hasOwnProperty('ok')) {
        this.ok = initObj.ok
      }
      else {
        this.ok = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type moveToPoseTcpResponse
    // Serialize message field [ok]
    bufferOffset = _serializer.int8(obj.ok, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type moveToPoseTcpResponse
    let len;
    let data = new moveToPoseTcpResponse(null);
    // Deserialize message field [ok]
    data.ok = _deserializer.int8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'mp_mini_picker/moveToPoseTcpResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '719c501bbbeb289704ee5d42501844db';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int8 ok
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new moveToPoseTcpResponse(null);
    if (msg.ok !== undefined) {
      resolved.ok = msg.ok;
    }
    else {
      resolved.ok = 0
    }

    return resolved;
    }
};

module.exports = {
  Request: moveToPoseTcpRequest,
  Response: moveToPoseTcpResponse,
  md5sum() { return 'c25f95a4c81298b09162de739af2fd0e'; },
  datatype() { return 'mp_mini_picker/moveToPoseTcp'; }
};
