// Auto-generated. Do not edit!

// (in-package kortex_tools.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class SaveDataRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.Save = null;
    }
    else {
      if (initObj.hasOwnProperty('Save')) {
        this.Save = initObj.Save
      }
      else {
        this.Save = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SaveDataRequest
    // Serialize message field [Save]
    bufferOffset = _serializer.int64(obj.Save, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SaveDataRequest
    let len;
    let data = new SaveDataRequest(null);
    // Deserialize message field [Save]
    data.Save = _deserializer.int64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_tools/SaveDataRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd27cc03f840c3e5aa15802dc4692ad7d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int64 Save
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SaveDataRequest(null);
    if (msg.Save !== undefined) {
      resolved.Save = msg.Save;
    }
    else {
      resolved.Save = 0
    }

    return resolved;
    }
};

class SaveDataResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.Success = null;
    }
    else {
      if (initObj.hasOwnProperty('Success')) {
        this.Success = initObj.Success
      }
      else {
        this.Success = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SaveDataResponse
    // Serialize message field [Success]
    bufferOffset = _serializer.int64(obj.Success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SaveDataResponse
    let len;
    let data = new SaveDataResponse(null);
    // Deserialize message field [Success]
    data.Success = _deserializer.int64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_tools/SaveDataResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f5ac40719150c2d44ad14cf750430183';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int64 Success
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SaveDataResponse(null);
    if (msg.Success !== undefined) {
      resolved.Success = msg.Success;
    }
    else {
      resolved.Success = 0
    }

    return resolved;
    }
};

module.exports = {
  Request: SaveDataRequest,
  Response: SaveDataResponse,
  md5sum() { return '1497dd37643804fd87f9722c47ea18be'; },
  datatype() { return 'kortex_tools/SaveData'; }
};
