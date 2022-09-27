
"use strict";

let ConfigMsg = require('./ConfigMsg.js');
let IMUDataMsg = require('./IMUDataMsg.js');
let ReconDataMsg = require('./ReconDataMsg.js');
let MotorListenMsg = require('./MotorListenMsg.js');
let UserChoiceMsg = require('./UserChoiceMsg.js');
let KillConfirmationMsg = require('./KillConfirmationMsg.js');
let MotorDataMsg = require('./MotorDataMsg.js');

module.exports = {
  ConfigMsg: ConfigMsg,
  IMUDataMsg: IMUDataMsg,
  ReconDataMsg: ReconDataMsg,
  MotorListenMsg: MotorListenMsg,
  UserChoiceMsg: UserChoiceMsg,
  KillConfirmationMsg: KillConfirmationMsg,
  MotorDataMsg: MotorDataMsg,
};
