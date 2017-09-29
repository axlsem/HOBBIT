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
 * @fileoverview Generating Python for HOBBIT blocks.
 * @author e0925419@student.tuwien.ac.at (Alexander Semeliker)
 */
'use strict';

goog.provide('Blockly.Python.hobbit');

goog.require('Blockly.Python');


Blockly.Python['hobbit_move'] = function(block) {
  var value_speed = Blockly.Python.valueToCode(block, 'speed', Blockly.Python.ORDER_ATOMIC);
  var dropdown_direction = block.getFieldValue('direction');
  var code = "";
  var message;
  
  Blockly.Python.definitions_['import_rospy'] = 'import rospy';
  Blockly.Python.definitions_['defPublisher'] = Blockly.Python.CreatePublisher();
  Blockly.Python.definitions_['rospy_init_node'] = 'rospy.init_node(\'demo\', anonymous=True)';
  
  Blockly.Python.UpdateImport('geometry_msgs',['Twist','Vector3']);
  code += "message = Twist(Vector3("+dropdown_direction+value_speed+",0,0),Vector3(0,0,0))\n";
  code += "MyPublisher(\'/cmd_vel\',message, Twist)\n";
 
  return code;
};

