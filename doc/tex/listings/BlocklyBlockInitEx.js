Blockly.Blocks['hobbit_arm_gripper'] = {
    init: function () {
        this.jsonInit({
            "type": "hobbit_arm_gripper",
            "message0": "%1 Gripper",
            "args0": [
                {
                    "type": "field_dropdown",
                    "name": "gripper_position",
                    "options": [
                        [
                            "Open",
                            "open"
                        ],
                        [
                            "Close",
                            "close"
                        ]
                    ]
                }
            ],
            "previousStatement": null,
            "nextStatement": null,
            "colour": 360,
            "tooltip": "Control HOBBIT's gripper",
            "helpUrl": ""
        });
    }
};