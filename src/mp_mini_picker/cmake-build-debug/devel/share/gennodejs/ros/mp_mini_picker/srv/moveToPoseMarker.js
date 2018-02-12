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

class moveToPoseMarkerRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.pose = null;
      this.tcpPmarker = null;
      this.tcpRmarker = null;
    }
    else {
      if (initObj.hasOwnProperty('pose')) {
        this.pose = initObj.pose
      }
      else {
        this.pose = new Array(6).fill(0);
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
    // Serializes a message object of type moveToPoseMarkerRequest
    // Check that the constant length array field [pose] has the right length
    if (obj.pose.length !== 6) {
      throw new Error('Unable to serialize array field pose - length must be 6')
    }
    // Serialize message field [pose]
    bufferOffset = _arraySerializer.float64(obj.pose, buffer, bufferOffset, 6);
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
    //deserializes a message object of type moveToPoseMarkerRequest
    let len;
    let data = new moveToPoseMarkerRequest(null);
    // Deserialize message field [pose]
    data.pose = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [tcpPmarker]
    data.tcpPmarker = _arrayDeserializer.float64(buffer, bufferOffset, 3)
    // Deserialize message field [tcpRmarker]
    data.tcpRmarker = _arrayDeserializer.float64(buffer, bufferOffset, 3)
    return data;
  }

  static getMessageSize(object) {
    return 96;
  }

  static datatype() {
    // Returns string type for a service object
    return 'mp_mini_picker/moveToPoseMarkerRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3e93f401cd80dbcc7b92bb016251f5d2';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64[6] pose
    float64[3] tcpPmarker
    float64[3] tcpRmarker
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new moveToPoseMarkerRequest(null);
    if (msg.pose !== undefined) {
      resolved.pose = msg.pose;
    }
    else {
      resolved.pose = new Array(6).fill(0)
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

class moveToPoseMarkerResponse {
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
    // Serializes a message object of type moveToPoseMarkerResponse
    // Serialize message field [ok]
    bufferOffset = _serializer.int8(obj.ok, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type moveToPoseMarkerResponse
    let len;
    let data = new moveToPoseMarkerResponse(null);
    // Deserialize message field [ok]
    data.ok = _deserializer.int8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'mp_mini_picker/moveToPoseMarkerResponse';
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
    const resolved = new moveToPoseMarkerResponse(null);
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
  Request: moveToPoseMarkerRequest,
  Response: moveToPoseMarkerResponse,
  md5sum() { return '90ff3d42ea6ed49171a1cd36008690f4'; },
  datatype() { return 'mp_mini_picker/moveToPoseMarker'; }
};
