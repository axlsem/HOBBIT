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
	var dropdown_direction = block.getFieldValue('direction');
	
	var code = "";
	var value_angle_deg = value_angle/180*Math.PI;

	Blockly.Python.InitROS();
	// Blockly.Python.definitions_['from_geometry_msgs.msg_import_Twist'] = 'from geometry_msgs.msg import Twist';

	// code += "\message = Twist()\n";
	// code += "message.angular.z = "+dropdown_direction+value_angle_deg+"\n";
	// code += Blockly.Python.NodeName+'.publishTopic(\'/cmd_vel\', \'Twist\', message)\n';
	code += Blockly.Python.NodeName+".turn("+dropdown_direction+value_angle_deg+")\n";

	return code;
};

Blockly.Python['hobbit_move'] = function(block) {
	var value_speed = Blockly.Python.valueToCode(block, 'speed', Blockly.Python.ORDER_ATOMIC);
	var dropdown_direction = block.getFieldValue('direction');
	var code = "";

	Blockly.Python.InitROS();
	// Blockly.Python.definitions_['from_geometry_msgs.msg_import_Twist'] = 'from geometry_msgs.msg import Twist';

	// code += "\message = Twist()\n";
	// code += "message.linear.x = "+dropdown_direction+value_speed+"\n";
	// code += Blockly.Python.NodeName+'.publishTopic(\'/cmd_vel\', \'Twist\', message)\n';
	
	code += Blockly.Python.NodeName+'.move('+dropdown_direction+value_speed+')\n';

	return code;
};

Blockly.Python['hobbit_undock'] = function(block) {
	// var code = "";

	Blockly.Python.InitROS();
	// Blockly.Python.definitions_['from_geometry_msgs.msg_import_Twist'] = 'from geometry_msgs.msg import Twist';

	// code += "\message = Twist()\n";
	// code += "message.linear.x = -0.5\n";
	// code += Blockly.Python.NodeName+'.publishTopic(\'/cmd_vel\', \'Twist\', message)\n';

	return Blockly.Python.NodeName+'.move(-0.4)\n';
};

Blockly.Python['hobbit_navigation_test'] = function(block) {
	var value_pos_x = Blockly.Python.valueToCode(block, 'pos_x', Blockly.Python.ORDER_ATOMIC);
	var value_pos_y = Blockly.Python.valueToCode(block, 'pos_y', Blockly.Python.ORDER_ATOMIC);
	var value_pos_z = Blockly.Python.valueToCode(block, 'pos_z', Blockly.Python.ORDER_ATOMIC);
	var value_quat_x = Blockly.Python.valueToCode(block, 'quat_x', Blockly.Python.ORDER_ATOMIC);
	var value_quat_y = Blockly.Python.valueToCode(block, 'quat_y', Blockly.Python.ORDER_ATOMIC);
	var value_quat_z = Blockly.Python.valueToCode(block, 'quat_z', Blockly.Python.ORDER_ATOMIC);
	var value_quat_w = Blockly.Python.valueToCode(block, 'quat_w', Blockly.Python.ORDER_ATOMIC);
	
	var code = "";

	Blockly.Python.InitROS();
	// Blockly.Python.definitions_['from_geometry_msgs.msg_import_PoseStamped'] = 'from geometry_msgs.msg import PoseStamped';

	// code += "\message = PoseStamped()\n";
	// code += "message.header.frame_id = \'map\'\n";
	// code += "message.header.stamp = rospy.Time.now()\n";
	// code += "message.pose.position.x = "+value_pos_x+"\n";
	// code += "message.pose.position.y = "+value_pos_y+"\n";
	// code += "message.pose.position.z = "+value_pos_z+"\n";
	// code += "message.pose.orientation.x = "+value_quat_x+"\n";
	// code += "message.pose.orientation.y = "+value_quat_y+"\n";
	// code += "message.pose.orientation.z = "+value_quat_z+"\n";
	// code += "message.pose.orientation.w = "+value_quat_w+"\n";
	
	// code += Blockly.Python.NodeName+'.publishTopic(\'/move_base_simple/goal\', \'PoseStamped\', message)\n';

	code += "position = ("+[value_pos_x,value_pos_y,value_pos_z].join(",")+")\n"
	code += "orientation = ("+[value_quat_x,value_quat_y,value_quat_z,value_quat_w].join(",")+")\n"
	code += Blockly.Python.NodeName+".navigate(position,orientation)\n"

	return code;
};

Blockly.Python['hobbit_head'] = function(block) {
	var dropdown_head_position = block.getFieldValue('head_position');

	Blockly.Python.InitROS();

	var code = "";
	code += "\message = \'" + dropdown_head_position.toString() + "\'\n";
	code += Blockly.Python.NodeName+'.publishTopic(\'/head/move\', \'String\', message)\n';

	return code;
};

Blockly.Python['hobbit_emo'] = function(block) {
	var dropdown_emotion = block.getFieldValue('emotion');

	Blockly.Python.InitROS();

	var code = "";
	code += "\message = \'" + dropdown_emotion.toString() + "\'\n";
	code += Blockly.Python.NodeName+'.publishTopic(\'/head/emo\', \'String\', message)\n';

	return code;
};

Blockly.Python['ROS_publisher'] = function(block) {
	var value_topic_name = Blockly.Python.valueToCode(block, 'topic_name', Blockly.Python.ORDER_ATOMIC);
	var value_message = Blockly.Python.valueToCode(block, 'message', Blockly.Python.ORDER_ATOMIC);
	var value_message_type = block.getFieldValue('message_type');

	Blockly.Python.InitROS();

	var code = "";
	code += "\message = " + value_message.toString() + "\n";
	code += Blockly.Python.NodeName+'.publishTopic(\''+value_topic_name+'\', \''+value_message_type+'\', message)\n';

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

Blockly.Python['hobbit_arm_move'] = function(block) {
	var dropdown_movement = block.getFieldValue('movement');
	var code = "";

	Blockly.Python.InitROS();
	
	code += Blockly.Python.NodeName+'.moveArm(\''+dropdown_movement+'\')\n';

	return code;
};

Blockly.Python['hobbit_arm_turntable'] = function(block) {
	var dropdown_movement = block.getFieldValue('movement');
	var code = "";

	Blockly.Python.InitROS();
	
	code += Blockly.Python.NodeName+'.moveArm(\''+dropdown_movement+'\')\n';

	return code;
};

Blockly.Python['hobbit_arm_gripper'] = function(block) {
	var dropdown_movement = block.getFieldValue('movement');
	var code = "";

	Blockly.Python.InitROS();
	
	code += Blockly.Python.NodeName+'.moveArm(\''+dropdown_movement+'\')\n';

	return code;
};
