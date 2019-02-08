function toIndex() {
    window.location.href = window.location.origin;
};

var blockId = window.location.search.substring(1);

var allHelps = {
    "hobbit_move": {
        title: "Move HOBBIT",
        description: "This blocks moves HOBBIT a given distance in the given direction",
        args: [
            {
                type: "input",
                description: "The distance HOBBIT should move in metres"
            },
            {
                type: "dropdown",
                description: "Direction in which HOBBIT should move.",
                options: [
                    {
                        name: "forward",
                        description: "Move HOBBIT forward"
                    },
                    {
                        name: "backward",
                        description: "Move HOBBIT backward"
                    }
                ]
            }
        ]
    },
    "hobbit_yes_no": {
        title: "Ask user a yes-no-question",
        description: "This blocks is used to ask the user a yes-no-question and delivers a boolean value in respect of the given option.",
        example: "The following example sets HOBBIT's emotion to 'happy' if the user clicks the 'yes' button on the tablet interface when asked the question 'Are you happy?'.",
        args: [
            {
                type: "input",
                description: "Question the user should be asked"
            },
            {
                type: "dropdown",
                description: "Possible answers",
                options: [
                    {
                        name: "yes",
                        description: "Output of the block is set to TRUE if user's answer is 'Yes'"
                    },
                    {
                        name: "no",
                        description: "Output of the block is set to TRUE if user's answer is 'No'"
                    }
                ]
            }
        ]
    },
    "hobbit_user_input": {
        title: "Ask user a question",
        description: "This blocks provides the answer of the user to the given question.",
        example: "In the following example the user is prompted to provide his or her name via the tablet interface. The input is stored into the variable 'username' and then concatenated to the salutation which then will be displayed on HOBBIT's tablet.",
        args: [
            {
                type: "input",
                description: "Question the user should be asked"
            }
        ]
    },
    "hobbit_show_info": {
        title: "Show info on HOBBIT's tablet",
        description: "This blocks displays a given text on HOBBIT's tablet.",
        args: [
            {
                type: "input",
                description: "Text, which should be displayed"
            }
        ]
    },
    "hobbit_show_info_confirm": {
        title: "Show info and wait for confirmation",
        description: "This blocks displays a given text on HOBBIT's tablet and waits for confirmation of the user. The the program continues only when the user confirms the shown message.",
        example: "In the following example first the user is asked to put an object on HOBBIT's turntable. The user has to confirm the action before the program continues and HOBBIT turns the turntable. After that the user is asked to remove the object and, again, confirm it. Only after the confirmation HOBBIT puts the turntable back to its storing position.",
        args: [
            {
                type: "input",
                description: "Text, which should be displayed"
            }
        ]
    },
    "hobbit_emo": {
        title: "Control HOBBIT's eyes",
        description: "This blocks sets HOBBIT's eyes to the given emotion.",
        args: [
            {
                type: "dropdown",
                description: "Emotion HOBBIT should show",
                options: [
                    {
                        name: "be happy",
                        description: "HOBBIT shows happy face"
                    },
                    {
                        name: "be very happy",
                        description: "HOBBIT shows very happy face"
                    },
                    {
                        name: "be little tired",
                        description: "HOBBIT shows little tired face"
                    },
                    {
                        name: "be very tired",
                        description: "HOBBIT shows very tired face"
                    },
                    {
                        name: "be sad",
                        description: "HOBBIT shows sad face"
                    },
                    {
                        name: "be concerned",
                        description: "HOBBIT shows concerned face"
                    },
                    {
                        name: "wonder",
                        description: "HOBBIT shows wondering face"
                    },
                    {
                        name: "be neutral",
                        description: "HOBBIT shows neutral face"
                    },
                    {
                        name: "sleep",
                        description: "HOBBIT sleeps"
                    }
                ]
            }
        ]
    },
    "hobbit_head": {
        title: "Move HOBBIT's head",
        description: "This blocks moves HOBBIT's head to the given position.",
        args: [
            {
                type: "dropdown",
                description: "Direction to which HOBBIT should look",
                options: [
                    {
                        name: "straight",
                        description: "HOBBIT looks straight"
                    },
                    {
                        name: "up",
                        description: "HOBBIT looks up"
                    },
                    {
                        name: "down",
                        description: "HOBBIT looks down"
                    },
                    {
                        name: "right",
                        description: "HOBBIT looks right"
                    },
                    {
                        name: "left",
                        description: "HOBBIT looks left"
                    },
                    {
                        name: "to upper right corner",
                        description: "HOBBIT looks to upper right corner"
                    },
                    {
                        name: "to upper left corner",
                        description: "HOBBIT looks to upper left corner"
                    },
                    {
                        name: "to lower right corner",
                        description: "HOBBIT looks to lower right corner"
                    },
                    {
                        name: "to lower left corner",
                        description: "HOBBIT looks to lower left corner"
                    },
                    {
                        name: "to grasp",
                        description: "HOBBIT looks to floor to grasp object"
                    },
                    {
                        name: "to turntable",
                        description: "HOBBIT looks to the turntable"
                    },
                    {
                        name: "to table",
                        description: "HOBBIT looks to the table in front"
                    },
                    {
                        name: "little down",
                        description: "HOBBIT looks little down"
                    }
                ]
            }
        ]
    },
    "hobbit_turn": {
        title: "Turn HOBBIT",
        description: "This blocks turns HOBBIT a given angle in the given direction",
        args: [
            {
                type: "input",
                description: "The angle HOBBIT should turn in degrees"
            },
            {
                type: "dropdown",
                description: "The direction in which HOBBIT should turn.",
                options: [
                    {
                        name: "left",
                        description: "Turn HOBBIT left from its point of view"
                    },
                    {
                        name: "right",
                        description: "Turn HOBBIT right from its point of view"
                    }
                ]
            }
        ]
    },
    "hobbit_arm_gripper": {
        title: "Control HOBBIT's gripper",
        description: "This blocks allows to open and close HOBBIT's gripper",
        args: [
            {
                type: "dropdown",
                description: "Position of gripper",
                options: [
                    {
                        name: "Open",
                        description: "Open gripper"
                    },
                    {
                        name: "Close",
                        description: "Close gripper"
                    }
                ]
            }
        ]
    },
    "hobbit_arm_move": {
        title: "Move HOBBIT's arm",
        description: "This blocks move HOBBIT's arm to the given position.",
        args: [
            {
                type: "dropdown",
                description: "Arm position",
                options: [
                    {
                        name: "candle position",
                        description: "Arm is fully stretched"
                    },
                    {
                        name: "home position",
                        description: "Arm is angled near the body"
                    },
                    {
                        name: "prepare grasping from floor",
                        description: "Put arm with gripper pointing down to ground to pick up an object"
                    },
                    {
                        name: "table",
                        description: "Move arm to table in front of HOBBIT"
                    },
                    {
                        name: "tray",
                        description: "Tray is used to place and carry object"
                    }
                ]
            }
        ]
    },
    "hobbit_arm_turntable": {
        title: "Use turntable",
        description: "This block provides the ability to grab, turn and store HOBBIT's turntable.",
        args: [
            {
                type: "dropdown",
                description: "Action to be performed with turntable",
                options: [
                    {
                        name: "Grab turntable",
                        description: "Grab turntable from storing position"
                    },
                    {
                        name: "Store turntable",
                        description: "Put turntable back to storing position"
                    },
                    {
                        name: "Turn turntable clockwise",
                        description: "Turn turntable clockwise"
                    },
                    {
                        name: "Turn turntable counterclockwise",
                        description: "Turn turntable counterclockwise"
                    }
                ]
            }
        ]
    },
    "hobbit_undock": {
        title: "Undock HOBBIT from charger",
        description: "This block undocks the robot from charger to get a clearer signal from the fron sensor."
    },
    "hobbit_navigation_test": {
        title: "HOBBIT navigation",
        description: "This block navigates HOBBIT to the given point in free space with the given orientation in free space in quaternion form.",
        args: [
            {
                type: "input",
                description: "x-coordinate of the point"
            },
            {
                type: "input",
                description: "y-coordinate of the point"
            },
            {
                type: "input",
                description: "z-coordinate of the point"
            },
            {
                type: "input",
                description: "x-component of the orientation"
            },
            {
                type: "input",
                description: "y-component of the orientation"
            },
            {
                type: "input",
                description: "z-component of the orientation"
            },
            {
                type: "input",
                description: "w-component of the orientation"
            }
        ]
    }
}
var sortedNames = Object.keys(allHelps).sort(function (a, b) {
    if (allHelps[a].title > allHelps[b].title) {
        return 1;
    }
    if (allHelps[a].title < allHelps[b].title) {
        return -1;
    }
    return 0
})

for (let block of sortedNames) {
    var isSelectedBlock = blockId == block;
    var x = document.createElement("A");
    x.setAttribute("href", "?" + block);
    x.setAttribute("class", "collection-item");
    x.setAttribute("style", "font-weight: normal;");
    if (isSelectedBlock) x.setAttribute("class", "collection-item active");
    x.innerHTML = allHelps[block].title;
    document.getElementById("blockList").appendChild(x);
}

var blockProps = allHelps[blockId];
if (blockProps) {
    $('#title').text(blockProps.title)
    $('#desc').text(blockProps.description)

    addImage('#block-image',"/img/blocks/" + blockId + ".png")

    if (blockProps.args) {
        for (let arg of blockProps.args) {
            $('#params').append('<li class="params-li"><span class="help-inp-type">' + arg.type.toUpperCase() + '</span>' + arg.description + '</li>')
            if (arg.type == "dropdown") {
                var opts = [];
                for (let option of arg.options) {
                    opts.push('<li class="params-li"><span class="help-dropdown-opt">' + option.name + '</span>' + option.description + '</li>')
                }
                $('#params').append('<ul class="nested-ul">' + opts.join("") + '</ul>')
            }
        }
    } else {
        $('#params').append('<li class="params-li">Block has no parameters</li>')
    }

    if (blockProps.example) {
        $('#exampledesc').text(blockProps.example)
        addImage('#help-example',"/img/blocks/ex-" + blockId + ".png")
    } else {
        $('#exampleheader').hide()
    }
} else {
    $('#paramsheader').hide()
    $('#exampleheader').hide()
    if (blockId) {
        $('#title').text('There is no help page for this block!')
    } else {
        $('#title').text('Overview')
        $('#desc').text('This page can be used to get further help for each block of the HOBBITS block set. Please select the desired block from the list on the left.')
    }
}

function addImage(cont, source) {
    var x = document.createElement("IMG");
    x.setAttribute("src", source);
    x.setAttribute("class", "responsive-img");
    x.setAttribute("onerror", "this.src='/img/blocks/default-thumb.png'");
    $(cont).append(x)
}