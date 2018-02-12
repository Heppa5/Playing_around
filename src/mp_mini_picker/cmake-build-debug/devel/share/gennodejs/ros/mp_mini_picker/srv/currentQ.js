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

class currentQRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.getQ = null;
    }
    else {
      if (initObj.hasOwnProperty('getQ')) {
        this.getQ = initObj.getQ
      }
      else {
        this.getQ = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type currentQRequest
    // Serialize message field [getQ]
    bufferOffset = _serializer.int8(obj.getQ, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type currentQRequest
    let len;
    let data = new currentQRequest(null);
    // Deserialize message field [getQ]
    data.getQ = _deserializer.int8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'mp_mini_picker/currentQRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '68064d5027f66825f7deba4e5b594236';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int8 getQ
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new currentQRequest(null);
    if (msg.getQ !== undefined) {
      resolved.getQ = msg.getQ;
    }
    else {
      resolved.getQ = 0
    }

    return resolved;
    }
};

class currentQResponse {
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
    // Serializes a message object of type currentQResponse
    // Check that the constant length array field [Q] has the right length
    if (obj.Q.length !== 6) {
      throw new Error('Unable to serialize array field Q - length must be 6')
    }
    // Serialize message field [Q]
    bufferOffset = _arraySerializer.float64(obj.Q, buffer, bufferOffset, 6);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type currentQResponse
    let len;
    let data = new currentQResponse(null);
    // Deserialize message field [Q]
    data.Q = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    return data;
  }

  static getMessageSize(object) {
    return 48;
  }

  static datatype() {
    // Returns string type for a service object
    return 'mp_mini_picker/currentQResponse';
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
    const resolved = new currentQResponse(null);
    if (msg.Q !== undefined) {
      resolved.Q = msg.Q;
    }
    else {
      resolved.Q = new Array(6).fill(0)
    }

    return resolved;
    }
};

module.exports = {
  Request: currentQRequest,
  Response: currentQResponse,
  md5sum() { return '0abad29c7af93c5a38bad143c421d80d'; },
  datatype() { return 'mp_mini_picker/currentQ'; }
};
