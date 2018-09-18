Blockly.Python['hobbit_arm_gripper'] = function(block) {
	var dropdown_movement = block.getFieldValue('gripper_position');

	return 'node.gripper(\''+dropdown_movement+'\')\n';
};