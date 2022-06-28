// Auto-generated. Do not edit!

// (in-package kortex_movement.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class cartesian_movementRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.x = null;
      this.y = null;
      this.z = null;
      this.thetax = null;
      this.thetay = null;
      this.thetaz = null;
    }
    else {
      if (initObj.hasOwnProperty('x')) {
        this.x = initObj.x
      }
      else {
        this.x = 0.0;
      }
      if (initObj.hasOwnProperty('y')) {
        this.y = initObj.y
      }
      else {
        this.y = 0.0;
      }
      if (initObj.hasOwnProperty('z')) {
        this.z = initObj.z
      }
      else {
        this.z = 0.0;
      }
      if (initObj.hasOwnProperty('thetax')) {
        this.thetax = initObj.thetax
      }
      else {
        this.thetax = 0.0;
      }
      if (initObj.hasOwnProperty('thetay')) {
        this.thetay = initObj.thetay
      }
      else {
        this.thetay = 0.0;
      }
      if (initObj.hasOwnProperty('thetaz')) {
        this.thetaz = initObj.thetaz
      }
      else {
        this.thetaz = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type cartesian_movementRequest
    // Serialize message field [x]
    bufferOffset = _serializer.float32(obj.x, buffer, bufferOffset);
    // Serialize message field [y]
    bufferOffset = _serializer.float32(obj.y, buffer, bufferOffset);
    // Serialize message field [z]
    bufferOffset = _serializer.float32(obj.z, buffer, bufferOffset);
    // Serialize message field [thetax]
    bufferOffset = _serializer.float32(obj.thetax, buffer, bufferOffset);
    // Serialize message field [thetay]
    bufferOffset = _serializer.float32(obj.thetay, buffer, bufferOffset);
    // Serialize message field [thetaz]
    bufferOffset = _serializer.float32(obj.thetaz, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type cartesian_movementRequest
    let len;
    let data = new cartesian_movementRequest(null);
    // Deserialize message field [x]
    data.x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [y]
    data.y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [z]
    data.z = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [thetax]
    data.thetax = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [thetay]
    data.thetay = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [thetaz]
    data.thetaz = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 24;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_movement/cartesian_movementRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '343f88ee256329e1ad031142d25c7292';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 x
    float32 y
    float32 z
    float32 thetax
    float32 thetay
    float32 thetaz
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new cartesian_movementRequest(null);
    if (msg.x !== undefined) {
      resolved.x = msg.x;
    }
    else {
      resolved.x = 0.0
    }

    if (msg.y !== undefined) {
      resolved.y = msg.y;
    }
    else {
      resolved.y = 0.0
    }

    if (msg.z !== undefined) {
      resolved.z = msg.z;
    }
    else {
      resolved.z = 0.0
    }

    if (msg.thetax !== undefined) {
      resolved.thetax = msg.thetax;
    }
    else {
      resolved.thetax = 0.0
    }

    if (msg.thetay !== undefined) {
      resolved.thetay = msg.thetay;
    }
    else {
      resolved.thetay = 0.0
    }

    if (msg.thetaz !== undefined) {
      resolved.thetaz = msg.thetaz;
    }
    else {
      resolved.thetaz = 0.0
    }

    return resolved;
    }
};

class cartesian_movementResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.output = null;
    }
    else {
      if (initObj.hasOwnProperty('output')) {
        this.output = initObj.output
      }
      else {
        this.output = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type cartesian_movementResponse
    // Serialize message field [output]
    bufferOffset = _serializer.bool(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type cartesian_movementResponse
    let len;
    let data = new cartesian_movementResponse(null);
    // Deserialize message field [output]
    data.output = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_movement/cartesian_movementResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd5fa62db5c86ed745052c3b25d12f430';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool output
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new cartesian_movementResponse(null);
    if (msg.output !== undefined) {
      resolved.output = msg.output;
    }
    else {
      resolved.output = false
    }

    return resolved;
    }
};

module.exports = {
  Request: cartesian_movementRequest,
  Response: cartesian_movementResponse,
  md5sum() { return '527d545043520e6207cc8929405c4e9e'; },
  datatype() { return 'kortex_movement/cartesian_movement'; }
};
