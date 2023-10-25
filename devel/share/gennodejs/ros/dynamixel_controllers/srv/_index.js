
"use strict";

let SetComplianceMargin = require('./SetComplianceMargin.js')
let TorqueEnable = require('./TorqueEnable.js')
let SetComplianceSlope = require('./SetComplianceSlope.js')
let SetSpeed = require('./SetSpeed.js')
let StopController = require('./StopController.js')
let RestartController = require('./RestartController.js')
let SetTorqueLimit = require('./SetTorqueLimit.js')
let StartController = require('./StartController.js')
let SetCompliancePunch = require('./SetCompliancePunch.js')

module.exports = {
  SetComplianceMargin: SetComplianceMargin,
  TorqueEnable: TorqueEnable,
  SetComplianceSlope: SetComplianceSlope,
  SetSpeed: SetSpeed,
  StopController: StopController,
  RestartController: RestartController,
  SetTorqueLimit: SetTorqueLimit,
  StartController: StartController,
  SetCompliancePunch: SetCompliancePunch,
};
