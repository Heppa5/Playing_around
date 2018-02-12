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

class moveToQRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.Q = null;
    }
    else {
      if (initObj.hasOwnProperty('Q')) {
        this.Q = initObj.Q
      }
      else {
        this.Q = new Array(6).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type moveToQRequest
    // Check that the constant length array field [Q] has the right length
    if (obj.Q.length !== 6) {
      throw new Error('Unable to serialize array field Q - length must be 6')
    }
    // Serialize message field [Q]
    bufferOffset = _arraySerializer.float64(obj.Q, buffer, bufferOffset, 6);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type moveToQRequest
    let len;
    let data = new moveToQRequest(null);
    // Deserialize message field [Q]
    data.Q = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    return data;
  }

  static getMessageSize(object) {
    return 48;
  }

  static datatype() {
    // Returns string type for a service object
    return 'mp_mini_picker/moveToQRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4317a35b45e376d0ab631a6538e289aa';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64[6] Q
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new moveToQRequest(null);
    if (msg.Q !== undefined) {
      resolved.Q = msg.Q;
    }
    else {
      resolved.Q = new Array(6).fill(0)
    }

    return resolved;
    }
};

class moveToQResponse {
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
    // Serializes a message object of type moveToQResponse
    // Serialize message field [ok]
    bufferOffset = _serializer.int8(obj.ok, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type moveToQResponse
    let len;
    let data = new moveToQResponse(null);
    // Deserialize message field [ok]
    data.ok = _deserializer.int8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'mp_mini_picker/moveToQResponse';
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
    const resolved = new moveToQResponse(null);
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
  Request: moveToQRequest,
  Response: moveToQResponse,
  md5sum() { return '1d1efbf9948db38dda60ea143a471260'; },
  datatype() { return 'mp_mini_picker/moveToQ'; }
};
