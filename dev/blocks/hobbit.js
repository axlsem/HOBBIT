/**
 * @license
 *
 * Copyright 2017 Automation and Control Institute (ACIN), TU Wien
 * http://www.acin.tuwien.ac.at/
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @fileoverview HOBBIT blocks for Blockly.
 * @author e0925419@student.tuwien.ac.at (Alexander Semeliker)
 */
'use strict';

goog.provide('Blockly.Blocks.hobbit');  // Deprecated
goog.provide('Blockly.Constants.hobbit');

goog.require('Blockly.Blocks');

/**
 * Common HSV hue for all blocks in this category.
 * @readonly
 */
Blockly.Constants.hobbit.HUE = 360;

Blockly.Blocks['hobbit_navigation_test'] = {
	init: function () {
		this.jsonInit({
			"type": "hobbit_navigation_test",
			"message0": "Navigate to x: %1 y: %2 z: %3 with orientation x: %4 y: %5 z: %6 w: %7",
			"args0": [
				{
					"type": "input_value",
					"name": "pos_x",
					"check": "Number",
					"align": "RIGHT"
				},
				{
					"type": "input_value",
					"name": "pos_y",
					"check": "Number",
					"align": "RIGHT"
				},
				{
					"type": "input_value",
					"name": "pos_z",
					"check": "Number",
					"align": "RIGHT"
				},
				{
					"type": "input_value",
					"name": "quat_x",
					"check": "Number",
					"align": "RIGHT"
				},
				{
					"type": "input_value",
					"name": "quat_y",
					"check": "Number",
					"align": "RIGHT"
				},
				{
					"type": "input_value",
					"name": "quat_z",
					"check": "Number",
					"align": "RIGHT"
				},
				{
					"type": "input_value",
					"name": "quat_w",
					"check": "Number",
					"align": "RIGHT"
				}
			],
			"inputsInline": false,
			"previousStatement": null,
			"nextStatement": null,
			"colour": Blockly.Constants.hobbit.HUE,
			"tooltip": "Test navigation.",
			"helpUrl": "",
			// "mutator": "hobbit_navigation_dialog"
		});
	}
};

Blockly.Constants.hobbit.HOBBIT_NAVIGATION_DIALOG_MUTATOR_MIXIN = {
	decompose: function (workspace) {
		// var topBlock = Blockly.Block.obtain(workspace, 'hobbit_navigation_test');
		// topBlock.initSvg();
		// console.log("Here comes the nav dialog");
		var topBlock = Blockly.Block.obtain(workspace, 'controls_if_if');
		topBlock.initSvg();
		return topBlock;
	},
	compose: function (topBlock) {
		// console.log(topBlock);
		// console.log("Now set nav xy!");
	},
	mutationToDom: function () {
		// var container = document.createElement('mutation');
		// var divisorInput = (this.getFieldValue('PROPERTY') == 'DIVISIBLE_BY');
		// container.setAttribute('divisor_input', divisorInput);
		// return container;
		// console.log("mutToDom");
	},
	domToMutation: function (xmlElement) {
		// var hasDivisorInput = (xmlElement.getAttribute('divisor_input') == 'true');
		// this.updateShape_(hasDivisorInput);  // Helper function for adding/removing 2nd input.
		// console.log("domToMut");
	}
};

Blockly.Extensions.registerMutator('hobbit_navigation_dialog',
	Blockly.Constants.hobbit.HOBBIT_NAVIGATION_DIALOG_MUTATOR_MIXIN);

Blockly.Blocks['hobbit_show_info_confirm'] = {
	init: function () {
		this.jsonInit({
			"type": "hobbit_show_info",
			"message0": "show info %1 and wait for confirmation",
			"args0": [
				{
					"type": "input_value",
					"name": "text",
					"check": "String"
				}
			],
			"inputsInline": true,
			"previousStatement": null,
			"nextStatement": null,
			"colour": Blockly.Constants.hobbit.HUE,
			"tooltip": "Show text on tablet and wait for confirmation.",
			"helpUrl": ""
		});
	}
};

Blockly.Blocks['hobbit_show_info'] = {
	init: function () {
		this.jsonInit({
			"type": "hobbit_show_info",
			"message0": "show info %1",
			"args0": [
				{
					"type": "input_value",
					"name": "text",
					"check": "String"
				}
			],
			"inputsInline": true,
			"previousStatement": null,
			"nextStatement": null,
			"colour": Blockly.Constants.hobbit.HUE,
			"tooltip": "Show text on tablet.",
			"helpUrl": ""
		});
	}
};

Blockly.Blocks['hobbit_user_input'] = {
	init: function () {
		this.jsonInit({
			"type": "hobbit_user_input",
			"message0": "user's response to %1",
			"args0": [
				{
					"type": "input_value",
					"name": "text",
					"check": "String"
				}
			],
			"inputsInline": true,
			"output": "String",
			"colour": Blockly.Constants.hobbit.HUE,
			"tooltip": "Get user's input.",
			"helpUrl": ""
		});
	}
};

Blockly.Blocks['hobbit_yes_no'] = {
	init: function () {
		this.jsonInit({
			"type": "hobbit_yes_no",
			"message0": "user responds to %1 with %2",
			"args0": [
				{
					"type": "input_value",
					"name": "text",
					"check": "String"
				},
				{
					"type": "field_dropdown",
					"name": "yes_no",
					"options": [
						[
							"yes",
							"D_YES"
						],
						[
							"no",
							"D_NO"
						]
					]
				}
			],
			"inputsInline": true,
			"output": "Boolean",
			"colour": Blockly.Constants.hobbit.HUE,
			"tooltip": "Ask a question and wait for user response (yes/no).",
			"helpUrl": ""
		});
	}
};

Blockly.Blocks['hobbit_call_service'] = {
	init: function () {
		this.jsonInit({
			"type": "hobbit_call_service",
			"message0": "response from service %1 from type %2 called with %3",
			"args0": [
				{
					"type": "input_value",
					"name": "service_name",
					"check": "String"
				},
				{
					"type": "input_value",
					"name": "service_type",
					"check": "String"
				},
				{
					"type": "field_dropdown",
					"name": "has_parameters",
					"options": [
						[
							"parameters",
							"params"
						],
						[
							"no parameters",
							"no_params"
						]
					]
				}
			],
			"inputsInline": true,
			"output": null,
			"colour": Blockly.Constants.hobbit.HUE,
			"tooltip": "Get response from service.",
			"helpUrl": "",
			"mutator": "hobbit_service_has_parameters"
		});
	}
};

Blockly.Constants.hobbit.SERVICE_HAS_PARAMETERS_MUTATOR_MIXIN = {
	/**
	 * Create XML to represent whether the 'divisorInput' should be present.
	 * @return {Element} XML storage element.
	 * @this Blockly.Block
	 */
	mutationToDom: function () {
		var container = document.createElement('mutation');
		var parametersInput = (this.getFieldValue('has_parameters') == "params");
		container.setAttribute('parameters_input', parametersInput);
		return container;
	},
	/**
	 * Parse XML to restore the 'divisorInput'.
	 * @param {!Element} xmlElement XML storage element.
	 * @this Blockly.Block
	 */
	domToMutation: function (xmlElement) {
		var parametersInput = (xmlElement.getAttribute('parameters_input') == 'true');
		this.updateShape_(parametersInput);
	},
	/**
	 * Modify this block to have (or not have) an input for 'is divisible by'.
	 * @param {boolean} divisorInput True if this block has a divisor input.
	 * @private
	 * @this Blockly.Block
	 */
	updateShape_: function (parameters) {
		// Add or remove a Value Input.
		var inputExists = this.getInput('service_parameters');
		if (parameters) {
			if (!inputExists) {
				this.appendValueInput('service_parameters');
			}
		} else if (inputExists) {
			this.removeInput('service_parameters');
		}
	}
};

Blockly.Constants.hobbit.SERVICE_HAS_PARAMETERS_MUTATOR_EXTENSION = function () {
	this.getField('has_parameters').setValidator(function (option) {
		var parametersInput = (option == 'parameters');
		this.sourceBlock_.updateShape_(parametersInput);
	});
};

Blockly.Extensions.registerMutator('hobbit_service_has_parameters',
	Blockly.Constants.hobbit.SERVICE_HAS_PARAMETERS_MUTATOR_MIXIN,
	Blockly.Constants.hobbit.SERVICE_HAS_PARAMETERS_MUTATOR_EXTENSION);

Blockly.Blocks['hobbit_turn'] = {
	init: function () {
		this.jsonInit({
			"type": "hobbit_turn",
			"message0": "turn %1 degrees %2",
			"args0": [
				{
					"type": "input_value",
					"name": "angle",
					"check": "Number"
				},
				{
					"type": "field_dropdown",
					"name": "direction",
					"options": [
						[
							"left",
							"+"
						],
						[
							"right",
							"-"
						]
					]
				}
			],
			"previousStatement": null,
			"nextStatement": null,
			"colour": Blockly.Constants.hobbit.HUE,
			"tooltip": "Turn HOBBIT",
			"helpUrl": ""
		});
	}
};

Blockly.Blocks['hobbit_move'] = {
	init: function () {
		this.jsonInit({
			"type": "hobbit_move",
			"message0": "move %1 metres %2",
			"args0": [
				{
					"type": "input_value",
					"name": "speed",
					"check": "Number"
				},
				{
					"type": "field_dropdown",
					"name": "direction",
					"options": [
						[
							"forward",
							"+"
						],
						[
							"backward",
							"-"
						]
					]
				}
			],
			"previousStatement": null,
			"nextStatement": null,
			"colour": Blockly.Constants.hobbit.HUE,
			"tooltip": "Move HOBBIT",
			"helpUrl": ""
		});
	}
};

Blockly.Blocks['hobbit_head'] = {
	init: function () {
		this.jsonInit({
			"type": "hobbit_head",
			"message0": "look %1",
			"args0": [
				{
					"type": "field_dropdown",
					"name": "head_position",
					"options": [
						[
							"straight",
							"center_center"
						],
						[
							"up",
							"up_center"
						],
						[
							"down",
							"down_center"
						],
						[
							"right",
							"center_right"
						],
						[
							"left",
							"center_left"
						],
						[
							"to upper right corner",
							"up_right"
						],
						[
							"to upper left corner",
							"up_left"
						],
						[
							"to lower right corner",
							"down_right"
						],
						[
							"to lower left corner",
							"down_left"
						],
						[
							"to grasp",
							"to_grasp"
						],
						[
							"to turntable",
							"to_turntable"
						],
						[
							"for table",
							"search_table"
						],
						[
							"little down",
							"littledown_center"
						]
					]
				}
			],
			"previousStatement": null,
			"nextStatement": null,
			"colour": Blockly.Constants.hobbit.HUE,
			"tooltip": "Move HOBBIT's head",
			"helpUrl": ""
		});
	}
};

Blockly.Blocks['hobbit_emo'] = {
	init: function () {
		this.jsonInit({
			"type": "hobbit_emo",
			"message0": "%1",
			"args0": [
				{
					"type": "field_dropdown",
					"name": "emotion",
					"options": [
						[
							"be happy",
							"HAPPY"
						],
						[
							"be very happy",
							"VHAPPY"
						],
						[
							"be little tired",
							"LTIRED"
						],
						[
							"be very tired",
							"VTIRED"
						],
						[
							"be concerned",
							"CONCERNED"
						],
						[
							"be sad",
							"SAD"
						],
						[
							"wonder",
							"WONDERING"
						],
						[
							"be neutral",
							"NEUTRAL"
						],
						[
							"sleep",
							"SLEEPING"
						]
					]
				}
			],
			"previousStatement": null,
			"nextStatement": null,
			"colour": Blockly.Constants.hobbit.HUE,
			"tooltip": "Show emotion",
			"helpUrl": ""
		});
	}
};

Blockly.Blocks['ROS_publisher'] = {
	init: function () {
		this.jsonInit({
			"message0": "Publish %1 from type %2 %3 to topic %4",
			"args0": [
				{
					"type": "input_value",
					"name": "message",
					"align": "RIGHT"
				},
				{
					"type": "field_dropdown",
					"name": "message_type",
					"options": [
						[
							"String",
							"String"
						],
						[
							"Bool",
							"Bool"
						],
						[
							"Float64",
							"Float64"
						],
						[
							"Int64",
							"Int64"
						],
						[
							"UInt64",
							"UInt64"
						]
					]
				},
				{
					"type": "input_dummy",
					"align": "RIGHT"
				},
				{
					"type": "input_value",
					"name": "topic_name",
					"check": "String",
					"align": "RIGHT"
				}
			],
			"inputsInline": true,
			"previousStatement": null,
			"nextStatement": null,
			"colour": Blockly.Constants.hobbit.HUE,
			"tooltip": "Create publisher for ROS topics",
			"helpUrl": ""
		});
	}
};

Blockly.Blocks['hobbit_arm_move'] = {
	init: function () {
		this.jsonInit({
			"type": "hobbit_arm_move",
			"message0": "Move arm to %1",
			"args0": [
				{
					"type": "field_dropdown",
					"name": "movement",
					"options": [
						[
							"candle position",
							"MoveToCandle"
						],
						[
							"home position",
							"MoveToHome"
						],
						[
							"prepare grasping from floor",
							"MoveToPreGraspFloor"
						],
						[
							"table",
							"MoveToPreGraspTable"
						],
						[
							"tray",
							"MoveToTray"
						]
					]
				}
			],
			"previousStatement": null,
			"nextStatement": null,
			"colour": Blockly.Constants.hobbit.HUE,
			"tooltip": "Move HOBBIT's arm",
			"helpUrl": ""
		});
	}
};

Blockly.Blocks['hobbit_arm_turntable'] = {
	init: function () {
		this.jsonInit({
			"type": "hobbit_arm_turntable",
			"message0": "%1",
			"args0": [
				{
					"type": "field_dropdown",
					"name": "movement",
					"options": [
						[
							"Grab turntable",
							"MoveToLearning"
						],
						[
							"Store turntable",
							"StoreTurntable"
						],
						[
							"Turn turntable clockwise",
							"TurnTurntableCW"
						],
						[
							"Turn turntable counterclockwise",
							"TurnTurntableCCW"
						]
					]
				}
			],
			"previousStatement": null,
			"nextStatement": null,
			"colour": Blockly.Constants.hobbit.HUE,
			"tooltip": "Move turntable",
			"helpUrl": ""
		});
	}
};

Blockly.Blocks['hobbit_arm_gripper'] = {
	init: function () {
		this.jsonInit({
			"type": "hobbit_arm_move",
			"message0": "%1 Gripper",
			"args0": [
				{
					"type": "field_dropdown",
					"name": "movement",
					"options": [
						[
							"Open",
							"OpenGripper"
						],
						[
							"Close",
							"CloseGripper"
						]
					]
				}
			],
			"previousStatement": null,
			"nextStatement": null,
			"colour": Blockly.Constants.hobbit.HUE,
			"tooltip": "Control HOBBIT's gripper",
			"helpUrl": ""
		});
	}
};