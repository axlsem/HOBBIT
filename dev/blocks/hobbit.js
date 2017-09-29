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
 * Should be the same as Blockly.Msg.TEXTS_HUE
 * @readonly
 */
Blockly.Constants.hobbit.HUE = 120;

Blockly.Blocks['hobbit_move'] = {
  init: function() {
    this.jsonInit({
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