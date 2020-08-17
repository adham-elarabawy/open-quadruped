// Auto-generated. Do not edit!

// (in-package open_quadruped.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class joint_angles {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.fl = null;
      this.fr = null;
      this.bl = null;
      this.br = null;
    }
    else {
      if (initObj.hasOwnProperty('fl')) {
        this.fl = initObj.fl
      }
      else {
        this.fl = [];
      }
      if (initObj.hasOwnProperty('fr')) {
        this.fr = initObj.fr
      }
      else {
        this.fr = [];
      }
      if (initObj.hasOwnProperty('bl')) {
        this.bl = initObj.bl
      }
      else {
        this.bl = [];
      }
      if (initObj.hasOwnProperty('br')) {
        this.br = initObj.br
      }
      else {
        this.br = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type joint_angles
    // Serialize message field [fl]
    bufferOffset = _arraySerializer.float32(obj.fl, buffer, bufferOffset, null);
    // Serialize message field [fr]
    bufferOffset = _arraySerializer.float32(obj.fr, buffer, bufferOffset, null);
    // Serialize message field [bl]
    bufferOffset = _arraySerializer.float32(obj.bl, buffer, bufferOffset, null);
    // Serialize message field [br]
    bufferOffset = _arraySerializer.float32(obj.br, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type joint_angles
    let len;
    let data = new joint_angles(null);
    // Deserialize message field [fl]
    data.fl = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [fr]
    data.fr = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [bl]
    data.bl = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [br]
    data.br = _arrayDeserializer.float32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.fl.length;
    length += 4 * object.fr.length;
    length += 4 * object.bl.length;
    length += 4 * object.br.length;
    return length + 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'open_quadruped/joint_angles';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'cc955e0566b06523084e350c65b2944e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32[] fl
    float32[] fr
    float32[] bl
    float32[] br
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new joint_angles(null);
    if (msg.fl !== undefined) {
      resolved.fl = msg.fl;
    }
    else {
      resolved.fl = []
    }

    if (msg.fr !== undefined) {
      resolved.fr = msg.fr;
    }
    else {
      resolved.fr = []
    }

    if (msg.bl !== undefined) {
      resolved.bl = msg.bl;
    }
    else {
      resolved.bl = []
    }

    if (msg.br !== undefined) {
      resolved.br = msg.br;
    }
    else {
      resolved.br = []
    }

    return resolved;
    }
};

module.exports = joint_angles;
