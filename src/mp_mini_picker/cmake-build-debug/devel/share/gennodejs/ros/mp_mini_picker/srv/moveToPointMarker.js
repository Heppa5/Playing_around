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

class moveToPointMarkerRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.point = null;
      this.tcpPmarker = null;
      this.tcpRmarker = null;
    }
    else {
      if (initObj.hasOwnProperty('point')) {
        this.point = initObj.point
      }
      else {
        this.point = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('tcpPmarker')) {
        this.tcpPmarker = initObj.tcpPmarker
      }
      else {
        this.tcpPmarker = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('tcpRmarker')) {
        this.tcpRmarker = initObj.tcpRmarker
      }
      else {
        this.tcpRmarker = new Array(3).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type moveToPointMarkerRequest
    // Check that the constant length array field [point] has the right length
    if (obj.point.length !== 3) {
      throw new Error('Unable to serialize array field point - length must be 3')
    }
    // Serialize message field [point]
    bufferOffset = _arraySerializer.float64(obj.point, buffer, bufferOffset, 3);
    // Check that the constant length array field [tcpPmarker] has the right length
    if (obj.tcpPmarker.length !== 3) {
      throw new Error('Unable to serialize array field tcpPmarker - length must be 3')
    }
    // Serialize message field [tcpPmarker]
    bufferOffset = _arraySerializer.float64(obj.tcpPmarker, buffer, bufferOffset, 3);
    // Check that the constant length array field [tcpRmarker] has the right length
    if (obj.tcpRmarker.length !== 3) {
      throw new Error('Unable to serialize array field tcpRmarker - length must be 3')
    }
    // Serialize message field [tcpRmarker]
    bufferOffset = _arraySerializer.float64(obj.tcpRmarker, buffer, bufferOffset, 3);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type moveToPointMarkerRequest
    let len;
    let data = new moveToPointMarkerRequest(null);
    // Deserialize message field [point]
    data.point = _arrayDeserializer.float64(buffer, bufferOffset, 3)
    // Deserialize message field [tcpPmarker]
    data.tcpPmarker = _arrayDeserializer.float64(buffer, bufferOffset, 3)
    // Deserialize message field [tcpRmarker]
    data.tcpRmarker = _arrayDeserializer.float64(buffer, bufferOffset, 3)
    return data;
  }

  static getMessageSize(object) {
    return 72;
  }

  static datatype() {
    // Returns string type for a service object
    return 'mp_mini_picker/moveToPointMarkerRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '13d0cac9b9ca66b317c4dff8be8ae51c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64[3] point
    float64[3] tcpPmarker
    float64[3] tcpRmarker
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new moveToPointMarkerRequest(null);
    if (msg.point !== undefined) {
      resolved.point = msg.point;
    }
    else {
      resolved.point = new Array(3).fill(0)
    }

    if (msg.tcpPmarker !== undefined) {
      resolved.tcpPmarker = msg.tcpPmarker;
    }
    else {
      resolved.tcpPmarker = new Array(3).fill(0)
    }

    if (msg.tcpRmarker !== undefined) {
      resolved.tcpRmarker = msg.tcpRmarker;
    }
    else {
      resolved.tcpRmarker = new Array(3).fill(0)
    }

    return resolved;
    }
};

class moveToPointMarkerResponse {
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
    // Serializes a message object of type moveToPointMarkerResponse
    // Serialize message field [ok]
    bufferOffset = _serializer.int8(obj.ok, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type moveToPointMarkerResponse
    let len;
    let data = new moveToPointMarkerResponse(null);
    // Deserialize message field [ok]
    data.ok = _deserializer.int8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'mp_mini_picker/moveToPointMarkerResponse';
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
    const resolved = new moveToPointMarkerResponse(null);
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
  Request: moveToPointMarkerRequest,
  Response: moveToPointMarkerResponse,
  md5sum() { return '861274133f98039e263a7bec0ea74796'; },
  datatype() { return 'mp_mini_picker/moveToPointMarker'; }
};
