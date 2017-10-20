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
  
  Blockly.Python.InitROS();  
  Blockly.Python.UpdateImport('geometry_msgs.msg',['Twist','Vector3']);
  
  code += "\nmessage = Twist()\n";
  code += "message.linear.x = "+dropdown_direction+value_speed+"\n";
  code += Blockly.Python.PublisherFctName+"(\'/cmd_vel\', message, Twist)\n";
  // code += "time.sleep("+Blockly.Python.TimeDelay+")\n";
 
  return code;
};

Blockly.Python['hobbit_head'] = function(block) {
  var dropdown_head_position = block.getFieldValue('head_position');
  
  Blockly.Python.InitROS();
  Blockly.Python.UpdateImport('std_msgs.msg',['String']);
  
  var code = "";
  code += "\nmessage = \"" + dropdown_head_position.toString() + "\"\n";
  code += Blockly.Python.PublisherFctName+"(\'/head/move\', message, String)\n";
  // code += "time.sleep("+Blockly.Python.TimeDelay+")\n";
	
  return code;
};

Blockly.Python['hobbit_emo'] = function(block) {
  var dropdown_emotion = block.getFieldValue('emotion');
  
  Blockly.Python.InitROS();
  Blockly.Python.UpdateImport('std_msgs.msg',['String']);
  
  var code = "";
  code += "\nmessage = \"" + dropdown_emotion.toString() + "\"\n";
  code += Blockly.Python.PublisherFctName+"(\'/head/emo\', message, String)\n";
  // code += "time.sleep("+Blockly.Python.TimeDelay+")\n";
  
  return code;
};

Blockly.Python['ROS_publisher'] = function(block) {
  var value_topic_name = Blockly.Python.valueToCode(block, 'topic_name', Blockly.Python.ORDER_ATOMIC);
  var value_message = Blockly.Python.valueToCode(block, 'message', Blockly.Python.ORDER_ATOMIC);
  var value_message_type = block.getFieldValue('message_type');
  
  Blockly.Python.InitROS();
  Blockly.Python.UpdateImport('std_msgs.msg',[value_message_type]);
  
  var code = "";
  code += "\nmessage = " + value_message.toString() + "\n";
  code += Blockly.Python.PublisherFctName+"("+value_topic_name.toString()+", message,"+value_message_type.toString()+")\n";
  // code += "time.sleep("+Blockly.Python.TimeDelay+")\n";

  return code;
};
