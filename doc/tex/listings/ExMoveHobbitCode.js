Blockly.Python['hobbit_move'] = function(block) {
	var value_distance = Blockly.Python.valueToCode(block, 'distance', Blockly.Python.ORDER_ATOMIC);
	var dropdown_direction = block.getFieldValue('direction');
	Blockly.Python.InitROS();
	
	var code = Blockly.Python.NodeName+'.move('+dropdown_direction+value_distance+')\n';
	return code;
};