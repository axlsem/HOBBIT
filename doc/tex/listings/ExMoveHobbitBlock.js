Blockly.Blocks['hobbit_move'] = {
	init: function () {
		this.jsonInit({
			"type": "hobbit_move",
			"message0": "move %1 metres %2",
			"args0": [
				{
					"type": "input_value",
					"name": "distance",
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