// Auto-generated. Do not edit!

// (in-package camera_robot_interaction.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class MovementRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.r11 = null;
      this.r12 = null;
      this.r13 = null;
      this.r14 = null;
      this.r21 = null;
      this.r22 = null;
      this.r23 = null;
      this.r24 = null;
      this.r31 = null;
      this.r32 = null;
      this.r33 = null;
      this.r34 = null;
      this.gripper = null;
      this.rotate = null;
      this.rest_pos = null;
    }
    else {
      if (initObj.hasOwnProperty('r11')) {
        this.r11 = initObj.r11
      }
      else {
        this.r11 = 0.0;
      }
      if (initObj.hasOwnProperty('r12')) {
        this.r12 = initObj.r12
      }
      else {
        this.r12 = 0.0;
      }
      if (initObj.hasOwnProperty('r13')) {
        this.r13 = initObj.r13
      }
      else {
        this.r13 = 0.0;
      }
      if (initObj.hasOwnProperty('r14')) {
        this.r14 = initObj.r14
      }
      else {
        this.r14 = 0.0;
      }
      if (initObj.hasOwnProperty('r21')) {
        this.r21 = initObj.r21
      }
      else {
        this.r21 = 0.0;
      }
      if (initObj.hasOwnProperty('r22')) {
        this.r22 = initObj.r22
      }
      else {
        this.r22 = 0.0;
      }
      if (initObj.hasOwnProperty('r23')) {
        this.r23 = initObj.r23
      }
      else {
        this.r23 = 0.0;
      }
      if (initObj.hasOwnProperty('r24')) {
        this.r24 = initObj.r24
      }
      else {
        this.r24 = 0.0;
      }
      if (initObj.hasOwnProperty('r31')) {
        this.r31 = initObj.r31
      }
      else {
        this.r31 = 0.0;
      }
      if (initObj.hasOwnProperty('r32')) {
        this.r32 = initObj.r32
      }
      else {
        this.r32 = 0.0;
      }
      if (initObj.hasOwnProperty('r33')) {
        this.r33 = initObj.r33
      }
      else {
        this.r33 = 0.0;
      }
      if (initObj.hasOwnProperty('r34')) {
        this.r34 = initObj.r34
      }
      else {
        this.r34 = 0.0;
      }
      if (initObj.hasOwnProperty('gripper')) {
        this.gripper = initObj.gripper
      }
      else {
        this.gripper = 0.0;
      }
      if (initObj.hasOwnProperty('rotate')) {
        this.rotate = initObj.rotate
      }
      else {
        this.rotate = false;
      }
      if (initObj.hasOwnProperty('rest_pos')) {
        this.rest_pos = initObj.rest_pos
      }
      else {
        this.rest_pos = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MovementRequest
    // Serialize message field [r11]
    bufferOffset = _serializer.float32(obj.r11, buffer, bufferOffset);
    // Serialize message field [r12]
    bufferOffset = _serializer.float32(obj.r12, buffer, bufferOffset);
    // Serialize message field [r13]
    bufferOffset = _serializer.float32(obj.r13, buffer, bufferOffset);
    // Serialize message field [r14]
    bufferOffset = _serializer.float32(obj.r14, buffer, bufferOffset);
    // Serialize message field [r21]
    bufferOffset = _serializer.float32(obj.r21, buffer, bufferOffset);
    // Serialize message field [r22]
    bufferOffset = _serializer.float32(obj.r22, buffer, bufferOffset);
    // Serialize message field [r23]
    bufferOffset = _serializer.float32(obj.r23, buffer, bufferOffset);
    // Serialize message field [r24]
    bufferOffset = _serializer.float32(obj.r24, buffer, bufferOffset);
    // Serialize message field [r31]
    bufferOffset = _serializer.float32(obj.r31, buffer, bufferOffset);
    // Serialize message field [r32]
    bufferOffset = _serializer.float32(obj.r32, buffer, bufferOffset);
    // Serialize message field [r33]
    bufferOffset = _serializer.float32(obj.r33, buffer, bufferOffset);
    // Serialize message field [r34]
    bufferOffset = _serializer.float32(obj.r34, buffer, bufferOffset);
    // Serialize message field [gripper]
    bufferOffset = _serializer.float32(obj.gripper, buffer, bufferOffset);
    // Serialize message field [rotate]
    bufferOffset = _serializer.bool(obj.rotate, buffer, bufferOffset);
    // Serialize message field [rest_pos]
    bufferOffset = _serializer.bool(obj.rest_pos, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MovementRequest
    let len;
    let data = new MovementRequest(null);
    // Deserialize message field [r11]
    data.r11 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [r12]
    data.r12 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [r13]
    data.r13 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [r14]
    data.r14 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [r21]
    data.r21 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [r22]
    data.r22 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [r23]
    data.r23 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [r24]
    data.r24 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [r31]
    data.r31 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [r32]
    data.r32 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [r33]
    data.r33 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [r34]
    data.r34 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [gripper]
    data.gripper = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [rotate]
    data.rotate = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [rest_pos]
    data.rest_pos = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 54;
  }

  static datatype() {
    // Returns string type for a service object
    return 'camera_robot_interaction/MovementRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5f15cf2b3ea3f6f2b16936c710d373fd';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 r11
    float32 r12
    float32 r13
    float32 r14
    float32 r21
    float32 r22
    float32 r23
    float32 r24
    float32 r31
    float32 r32
    float32 r33
    float32 r34
    float32 gripper
    bool rotate
    bool rest_pos
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new MovementRequest(null);
    if (msg.r11 !== undefined) {
      resolved.r11 = msg.r11;
    }
    else {
      resolved.r11 = 0.0
    }

    if (msg.r12 !== undefined) {
      resolved.r12 = msg.r12;
    }
    else {
      resolved.r12 = 0.0
    }

    if (msg.r13 !== undefined) {
      resolved.r13 = msg.r13;
    }
    else {
      resolved.r13 = 0.0
    }

    if (msg.r14 !== undefined) {
      resolved.r14 = msg.r14;
    }
    else {
      resolved.r14 = 0.0
    }

    if (msg.r21 !== undefined) {
      resolved.r21 = msg.r21;
    }
    else {
      resolved.r21 = 0.0
    }

    if (msg.r22 !== undefined) {
      resolved.r22 = msg.r22;
    }
    else {
      resolved.r22 = 0.0
    }

    if (msg.r23 !== undefined) {
      resolved.r23 = msg.r23;
    }
    else {
      resolved.r23 = 0.0
    }

    if (msg.r24 !== undefined) {
      resolved.r24 = msg.r24;
    }
    else {
      resolved.r24 = 0.0
    }

    if (msg.r31 !== undefined) {
      resolved.r31 = msg.r31;
    }
    else {
      resolved.r31 = 0.0
    }

    if (msg.r32 !== undefined) {
      resolved.r32 = msg.r32;
    }
    else {
      resolved.r32 = 0.0
    }

    if (msg.r33 !== undefined) {
      resolved.r33 = msg.r33;
    }
    else {
      resolved.r33 = 0.0
    }

    if (msg.r34 !== undefined) {
      resolved.r34 = msg.r34;
    }
    else {
      resolved.r34 = 0.0
    }

    if (msg.gripper !== undefined) {
      resolved.gripper = msg.gripper;
    }
    else {
      resolved.gripper = 0.0
    }

    if (msg.rotate !== undefined) {
      resolved.rotate = msg.rotate;
    }
    else {
      resolved.rotate = false
    }

    if (msg.rest_pos !== undefined) {
      resolved.rest_pos = msg.rest_pos;
    }
    else {
      resolved.rest_pos = false
    }

    return resolved;
    }
};

class MovementResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.Success = null;
      this.gripper_fb = null;
    }
    else {
      if (initObj.hasOwnProperty('Success')) {
        this.Success = initObj.Success
      }
      else {
        this.Success = 0;
      }
      if (initObj.hasOwnProperty('gripper_fb')) {
        this.gripper_fb = initObj.gripper_fb
      }
      else {
        this.gripper_fb = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MovementResponse
    // Serialize message field [Success]
    bufferOffset = _serializer.int64(obj.Success, buffer, bufferOffset);
    // Serialize message field [gripper_fb]
    bufferOffset = _serializer.float32(obj.gripper_fb, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MovementResponse
    let len;
    let data = new MovementResponse(null);
    // Deserialize message field [Success]
    data.Success = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [gripper_fb]
    data.gripper_fb = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 12;
  }

  static datatype() {
    // Returns string type for a service object
    return 'camera_robot_interaction/MovementResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0baab5e2da097063283a0661272a622e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int64 Success
    float32 gripper_fb
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new MovementResponse(null);
    if (msg.Success !== undefined) {
      resolved.Success = msg.Success;
    }
    else {
      resolved.Success = 0
    }

    if (msg.gripper_fb !== undefined) {
      resolved.gripper_fb = msg.gripper_fb;
    }
    else {
      resolved.gripper_fb = 0.0
    }

    return resolved;
    }
};

module.exports = {
  Request: MovementRequest,
  Response: MovementResponse,
  md5sum() { return '4787e27e3721f1f2004cf17997b3e3ae'; },
  datatype() { return 'camera_robot_interaction/Movement'; }
};
