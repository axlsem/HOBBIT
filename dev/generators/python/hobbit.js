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


Blockly.Python['hobbit_turn'] = function(block) {
	var value_angle = Blockly.Python.valueToCode(block, 'angle', Blockly.Python.ORDER_ATOMIC);
	var code = "";
	var message;
	var value_angle_deg = value_angle/Math.PI*180;
	Blockly.Python.InitROS();

	code += "\message = Twist()\n";
	code += "message.angular.z = "+value_angle_deg+"\n";
	code += Blockly.Python.NodeName+'.publishTopic(\'/cmd_vel\', Twist, message)\n';

	return code;
};

Blockly.Python['hobbit_move'] = function(block) {
	var value_speed = Blockly.Python.valueToCode(block, 'speed', Blockly.Python.ORDER_ATOMIC);
	var dropdown_direction = block.getFieldValue('direction');
	var code = "";
	var message;

	Blockly.Python.InitROS();  

	code += "\message = Twist()\n";
	code += "message.linear.x = "+dropdown_direction+value_speed+"\n";
	code += Blockly.Python.NodeName+'.publishTopic(\'/cmd_vel\', Twist, message)\n';

	return code;
};

Blockly.Python['hobbit_head'] = function(block) {
	var dropdown_head_position = block.getFieldValue('head_position');

	Blockly.Python.InitROS();

	var code = "";
	code += "\message = \'" + dropdown_head_position.toString() + "\'\n";
	code += Blockly.Python.NodeName+'.publishTopic(\'/head/move\', String, message)\n';

	return code;
};

Blockly.Python['hobbit_emo'] = function(block) {
	var dropdown_emotion = block.getFieldValue('emotion');

	Blockly.Python.InitROS();

	var code = "";
	code += "\message = \'" + dropdown_emotion.toString() + "\'\n";
	code += Blockly.Python.NodeName+'.publishTopic(\'/head/emo\', String, message)\n';

	return code;
};

Blockly.Python['ROS_publisher'] = function(block) {
	var value_topic_name = Blockly.Python.valueToCode(block, 'topic_name', Blockly.Python.ORDER_ATOMIC);
	var value_message = Blockly.Python.valueToCode(block, 'message', Blockly.Python.ORDER_ATOMIC);
	var value_message_type = block.getFieldValue('message_type');

	Blockly.Python.InitROS();

	var code = "";
	code += "\message = " + value_message.toString() + "\n";
	code += Blockly.Python.NodeName+'.publishTopic('+value_topic_name+', '+value_message_type+', message)\n';

	return code;
};

Blockly.Python['hobbit_yes_no'] = function(block) {
	var value_text = Blockly.Python.valueToCode(block, 'text', Blockly.Python.ORDER_ATOMIC);
	var dropdown_yes_no = block.getFieldValue('yes_no');
	
	Blockly.Python.InitROS();

	var code = "";
	code += Blockly.Python.NodeName+'.askYesNoQuestion('+value_text+',\''+dropdown_yes_no+'\')';

	return [code, Blockly.Python.ORDER_NONE];
};

Blockly.Python['hobbit_user_input'] = function(block) {
	var value_text = Blockly.Python.valueToCode(block, 'text', Blockly.Python.ORDER_ATOMIC);
	
	Blockly.Python.InitROS();

	var code = "";
	code += Blockly.Python.NodeName+'.getUserInput('+value_text+')';

	return [code, Blockly.Python.ORDER_NONE];
};

Blockly.Python['hobbit_show_info'] = function(block) {
	var value_text = Blockly.Python.valueToCode(block, 'text', Blockly.Python.ORDER_ATOMIC);
	
	Blockly.Python.InitROS();

	var code = "";
	code += Blockly.Python.NodeName+'.showInfo('+value_text+')\n';

	return code;
};

Blockly.Python['hobbit_show_info_confirm'] = function(block) {
	var value_text = Blockly.Python.valueToCode(block, 'text', Blockly.Python.ORDER_ATOMIC);
	
	Blockly.Python.InitROS();

	var code = "";
	code += Blockly.Python.NodeName+'.showInfoOK('+value_text+')\n';

	return code;
};

Blockly.Python['hobbit_call_service'] = function(block) {
	var value_service_name = Blockly.Python.valueToCode(block, 'service_name', Blockly.Python.ORDER_ATOMIC);
	var value_service_type = Blockly.Python.valueToCode(block, 'service_type', Blockly.Python.ORDER_ATOMIC);
	var dropdown_has_parameters = block.getFieldValue('has_parameters');
	var value_service_parameters = Blockly.Python.valueToCode(block, 'service_parameters', Blockly.Python.ORDER_ATOMIC);
	
	if (dropdown_has_parameters == 'no_params') {
		value_service_parameters = 'None';
	}
	
	Blockly.Python.InitROS();

	var code = "";
	code += Blockly.Python.NodeName+'.callService('+value_service_name+','+value_service_type+','+value_service_parameters+')\n';

	return [code, Blockly.Python.ORDER_NONE];
};
