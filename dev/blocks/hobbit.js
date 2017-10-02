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
Blockly.Constants.hobbit.HUE = 120;

Blockly.Blocks['hobbit_move'] = {
  init: function() {
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
			  ""
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
  init: function() {
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
  init: function() {
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
  init: function() {
    this.jsonInit({
      "message0": "PUBLISH %1 topic name %2 message %3 message type %4",
      "args0": [
		{
		  "type": "input_dummy"
		},
		{
		  "type": "input_value",
		  "name": "topic_name",
		  "align": "RIGHT"
		},
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
		}
      ],
	  "inputsInline": false,
      "previousStatement": null,
	  "nextStatement": null,
	  "colour": Blockly.Constants.hobbit.HUE,
	  "tooltip": "Create publisher for ROS topics",
	  "helpUrl": ""
    });
  }
};
