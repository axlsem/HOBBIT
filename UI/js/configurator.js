var selectedType = 'Topic';
var inputCnt = 1;
var srvFieldCnt = 1;
typeListener();
init();
document.getElementById('typeSelector').addEventListener('change', typeListener);
var editMode = false;
var currentBlock = {};

// document.getElementById("title").value = "Move arm";
// document.getElementById("in1").value = "Position";
// document.getElementById("action").value = "hobbit_arm";
// document.getElementById("actiontype").value = "hobbit_msgs/ArmServerAction";
// document.getElementById("goal").value = "goal = ArmServerGoal()\ngoal.command.data = $1$\ngoal.velocity = 0.0\ngoal.joints = []";
// document.getElementById("pckImports").value = "from hobbit_msgs.msg import ArmServerGoal\nfrom hobbit_msgs.msg import ArmServerAction";
// document.getElementById("typeSelector").selectedIndex = 2;
// document.getElementById("donecb").value = "if status==1:\n\tprint status\nelse:\n\tprint result";
// typeListener();

var uniqueId = function () {
    return Math.random().toString(36).substr(2, 16);
};

$.put = function (url, data, callback, type) {

    if ($.isFunction(data)) {
        type = type || callback,
            callback = data,
            data = {}
    }

    return $.ajax({
        url: url,
        type: 'PUT',
        success: callback,
        data: data,
        contentType: type
    });
}

$.delete = function (url, data, callback, type) {

    if ($.isFunction(data)) {
        type = type || callback,
            callback = data,
            data = {}
    }

    return $.ajax({
        url: url,
        type: 'DELETE',
        success: callback,
        data: data,
        contentType: type
    });
}

function toggleEditMode(b) {
    var editBtn = document.getElementById("editBtn");
    editBtn.innerHTML = editMode ? "Edit" : "Cancel";
    editMode = !b;

    if (!editMode) {
        setFormValues(currentBlock.meta);
    }

    lockInputs(b);
}

function setElementVisible(elem, visible) {
    var display = visible ? 'block' : 'none';
    var shape = document.getElementById(elem);
    shape.setAttribute("style", "display:" + display + ";");
}

function init() {
    $.get('/block/list', function (data, status) {
        if (status == "success") {

            var selectedBlock = window.location.search.substring(1);
            allBlocks = data;

            for (var block of data) {
                var isSelectedBlock = block.id == selectedBlock;
                var x = document.createElement("A");
                x.setAttribute("href", "?" + block.id);
                x.setAttribute("class", "collection-item");
                x.setAttribute("style", "font-weight: normal;");
                if (isSelectedBlock) x.setAttribute("class", "collection-item active");
                x.innerHTML = block.meta.title;
                document.getElementById("blockList").appendChild(x);

                if (selectedBlock != "" && isSelectedBlock) {
                    setFormValues(block.meta, true);
                    if (block.meta.inputs) inputCnt = block.meta.inputs.length;
                    if (block.meta.fields) srvFieldCnt = block.meta.fields.length;
                    currentBlock = block;
                }
            }

            if (selectedBlock == "") {
                setElementVisible("editBtn", false);
                setElementVisible("saveBtn", false);
                setElementVisible("deleteBtn", false);
                setElementVisible("createBtn", true);
                lockInputs(false);
            } else {
                lockInputs(true);
            }

        } else {
            alert("Something went wrong!");
        }
    });
}

function lockInputs(lock) {
    var shapes = ["title", "tooltip", "typeSelector", "topic", "service", "msgtype", "srvtype", "message", "pckImports", "goal", "action", "actiontype", "actiontimeout", "donecb", "feedbackcb", "activecb","serviceout"];
    var inputShapes = Array.from({ length: inputCnt }, (v, k) => "in" + (k + 1).toString());
    var srvFieldShapes = Array.from({ length: srvFieldCnt }, (v, k) => "srvField" + (k + 1).toString());
    var srvValShapes = Array.from({ length: srvFieldCnt }, (v, k) => "srvMsgVal" + (k + 1).toString());
    var srvCodeShapes = Array.from({ length: srvFieldCnt }, (v, k) => "srvCode" + (k + 1).toString());

    shapes = shapes.concat(inputShapes).concat(srvFieldShapes).concat(srvValShapes).concat(srvCodeShapes);

    for (var shape of shapes) {
        document.getElementById(shape).disabled = lock;
    }

    var toggleBtns = document.getElementsByClassName("toggle");
    for (var btn of toggleBtns) {
        btn.disabled = lock;
    }

    var saveBtn = document.getElementById("saveBtn");
    saveBtn.disabled = lock;
    var deleteBtn = document.getElementById("deleteBtn");
    deleteBtn.disabled = lock;
}

function setFormValues(values, init) {
    var inputDiff = inputCnt - values.inputs.length;

    if (inputDiff < 0) {
        for (var i = 0; i < -inputDiff; i++) {
            addInput();
        }
    } else if (inputDiff > 0) {
        for (var i = 0; i < inputDiff; i++) {
            removeInput();
        }
    }


    document.getElementById("title").value = values.title;
    document.getElementById("tooltip").value = values.tooltip;
    document.getElementById("typeSelector").selectedIndex = ["Topic", "Service", "Action"].indexOf(values.type);

    if (values.pyImports) document.getElementById("pckImports").value = values.pyImports;
    else document.getElementById("pckImports").value = null;

    typeListener();

    for (var i = 0; i < values.inputs.length; i++) {
        document.getElementById("in" + (i + 1).toString()).value = values.inputs[i];
    }

    if (values.type == "Topic") {
        document.getElementById("message").value = values.message;
        document.getElementById("topic").value = values.topic;
        document.getElementById("msgtype").value = values.msgtype;
    } else if (values.type == "Service") {
        document.getElementById("service").value = values.service;
        document.getElementById("srvtype").value = values.srvtype;
        document.getElementById("serviceout").checked = values.hasOutput;

        var srvFiledDiff = srvFieldCnt - values.fields.length;

        if (srvFiledDiff < 0) {
            for (var i = 0; i < -srvFiledDiff; i++) {
                addSrvField();
            }
        } else if (srvFiledDiff > 0) {
            for (var i = 0; i < srvFiledDiff; i++) {
                removeSrvField();
            }
        }

        for (var i = 0; i < values.fields.length; i++) {
            document.getElementById("srvField" + (i + 1).toString()).value = values.fields[i].name;
            document.getElementById("srvMsgVal" + (i + 1).toString()).value = values.fields[i].value;
            document.getElementById("srvCode" + (i + 1).toString()).checked = values.fields[i].isCode;
        }

    } else if (values.type == "Action") {
        document.getElementById("goal").value = values.goal;
        document.getElementById("action").value = values.action;
        document.getElementById("actiontype").value = values.actiontype;
        document.getElementById("activecb").value = values.activecb;
        document.getElementById("donecb").value = values.donecb;
        document.getElementById("feedbackcb").value = values.feedbackcb;
    }
}

function toIndex() {
    window.location.href = window.location.origin;
};

function clearform() {
    window.location = window.location.origin + window.location.pathname;
}

function typeListener() {
    var typeSelector = document.getElementById("typeSelector");
    selectedType = typeSelector.options[typeSelector.selectedIndex].text;
    var elem = document.getElementById("config" + selectedType);
    elem.style.display = 'block';

    if (selectedType == 'Service') {
        $('#respout').show();
    } else {
        $('#respout').hide();
    }

    for (i = 0; i < typeSelector.length; i++) {
        if (i != typeSelector.selectedIndex) {
            var t = typeSelector.options[i].text;
            var elem = document.getElementById("config" + t);
            elem.style.display = 'none';
        }
    }
}

function getBlockDefintion() {
    var title = document.getElementById("title").value + " %1";
    var inputs = document.getElementById("inputs").getElementsByTagName("INPUT");
    var tooltip = document.getElementById("tooltip").value;

    var block = {};
    block["args0"] = [{ type: "input_dummy" }];

    var inpMessages = [];

    var inCnt = 1;
    for (var inp of inputs) {
        inCnt += 1;
        inpMessages.push(inp.value + " %" + inCnt.toString());
        var tmpInp = { type: "input_value", align: "RIGHT", name: "var" + inCnt.toString() };
        block.args0.push(tmpInp);
    }

    block.message0 = [title, inpMessages.join(" ")].join(" ");
    block.colour = 50;
    block.tooltip = tooltip;
    block.helpUrl = "";
    if (document.getElementById('serviceout').checked) {
        block.output = null;
    } else {
        block.previousStatement = null;
        block.nextStatement = null;
    }

    return block
}

function getCodeTopic(block) {
    var topic = document.getElementById("topic").value;
    var msgtypeRaw = document.getElementById("msgtype").value;
    var message = document.getElementById("message").value;

    var msgPackage = msgtypeRaw.split("/")[0];
    var msgtype = msgtypeRaw.split("/")[1];

    var importCode = [importSyntax(msgPackage + ".msg", msgtype)];
    importCode = importCode.concat(importExtraPackages()).join("");

    var varInits = [];
    for (var inCnt = 2; inCnt <= block.args0.length; inCnt++) {
        varInits.push("var value_var" + inCnt.toString() + "=Blockly.Python.valueToCode(block,\'var" + inCnt.toString() + "\', Blockly.Python.ORDER_ATOMIC);");
    }
    varInits.push("Blockly.Python.InitROS();");


    var re = /\$([0-9]+)\$/gm;
    while ((match = re.exec(message)) != null) {
        if (inCnt < parseInt(match[1]) + 2) {
            alert("Only " + (inCnt - 2).toString() + " inputs defined, but more used in message!")
            return ""
        }
        message = message.replace(match[0], "'+value_var" + (parseInt(match[1]) + 1).toString() + "+'");
    }

    message = "var code='\\n';" + message.split("\n").map(e => "code+=\'" + e + "\\n\'").join(";");
    var MsgImport = "code+='Hobbitlib.importMsg(\\'" + msgPackage + ".msg\\',\\'" + msgtype + "\\')\\n'";
    var pubCode = "code+=Blockly.Python.NodeName+'.publishTopic(\\'" + topic + "\\', \\'" + msgtype + "\\', message)\\n';"

    var pyCode = [message, MsgImport, pubCode].join(";");

    return [varInits.join(""), importCode, pyCode, "return code"].join("")
}

function getMetaInfoTopic() {
    var inputs = [];
    var inputElems = document.getElementById("inputs").getElementsByTagName("INPUT");
    for (var inp of inputElems) {
        inputs.push(inp.value)
    }

    return {
        type: "Topic",
        topic: document.getElementById("topic").value,
        msgtype: document.getElementById("msgtype").value,
        message: document.getElementById("message").value,
        title: document.getElementById("title").value,
        inputs: inputs,
        pyImports: document.getElementById("pckImports").value,
        tooltip: document.getElementById("tooltip").value

    }
}

function getMetaInfoAction() {
    var inputs = [];
    var inputElems = document.getElementById("inputs").getElementsByTagName("INPUT");
    for (var inp of inputElems) {
        inputs.push(inp.value)
    }

    return {
        type: "Action",
        actiontimeout: document.getElementById("actiontimeout").value,
        action: document.getElementById("action").value,
        actiontype: document.getElementById("actiontype").value,
        goal: document.getElementById("goal").value,
        donecb: document.getElementById("donecb").value,
        activecb: document.getElementById("activecb").value,
        feedbackcb: document.getElementById("feedbackcb").value,
        title: document.getElementById("title").value,
        inputs: inputs,
        pyImports: document.getElementById("pckImports").value,
        tooltip: document.getElementById("tooltip").value
    }
}

function getMetaInfoService() {
    var inputs = [];
    var inputElems = document.getElementById("inputs").getElementsByTagName("INPUT");
    for (var inp of inputElems) {
        inputs.push(inp.value)
    }

    return {
        type: "Service",
        service: document.getElementById("service").value,
        srvtype: document.getElementById("srvtype").value,
        title: document.getElementById("title").value,
        fields: formatSrvFields(),
        inputs: inputs,
        pyImports: document.getElementById("pckImports").value,
        tooltip: document.getElementById("tooltip").value,
        hasOutput: document.getElementById("serviceout").checked

    }
}

function importSyntax(msgPackage, msgtype) {
    var imp = ["from", msgPackage, "import", msgtype];
    var importCode = "Blockly.Python.definitions_['" + imp.join("_") + "']='" + imp.join(" ") + "';";

    return importCode
}

function importExtraPackages() {
    var pyImports = document.getElementById("pckImports").value.split("\n");

    return pyImports.map(v => "Blockly.Python.definitions_['" + v.replace(/ /g, "_") + "']='" + v + "';")

}

function replaceDollars(message, inCnt) {
    var re = /\$([0-9]+)\$/gm;
    while ((match = re.exec(message)) != null) {
        if (inCnt < parseInt(match[1]) + 2) {
            alert("Only " + (inCnt - 2).toString() + " inputs defined, but more used in message!")
            return ""
        }
        message = message.replace(match[0], "'+value_var" + (parseInt(match[1]) + 1).toString() + "+'");
    }
    return message
}

function formatSrvFields() {
    var srvFields = document.getElementById("requestFields").getElementsByClassName("srvField");
    var srvChBoxes = document.getElementById("requestFields").getElementsByClassName("srvCode");
    var fieldValues = document.getElementById("requestFields").getElementsByTagName("TEXTAREA");

    var msgNames = Array.prototype.map.call(srvFields, elem => elem.value);
    var msgValues = Array.prototype.map.call(fieldValues, elem => elem.value);
    var codeFlags = Array.prototype.map.call(srvChBoxes, elem => elem.checked);

    var fields = [];
    for (i = 0; i < msgNames.length; i++) {
        let temp = {
            name: msgNames[i],
            value: msgValues[i],
            isCode: codeFlags[i]
        };
        fields.push(temp);
    }

    return fields
}

function getCodeService(block, blockId) {
    var service = document.getElementById("service").value;
    var srvtypeRaw = document.getElementById("srvtype").value;
    var srvFields = document.getElementById("requestFields").getElementsByClassName("srvField");
    var srvChBoxes = document.getElementById("requestFields").getElementsByClassName("srvCode");
    var fieldValues = document.getElementById("requestFields").getElementsByTagName("TEXTAREA");
    var useResponse = document.getElementById('serviceout').checked;

    if (useResponse) {
        var func = "Blockly.Python.definitions_[\'%" + blockId + "\'] = \'def srv" + blockId + "():\\n\'"
        var addto = "func";
        var resp = "'\\treturn '+";
        var indent = "\\t";
    } else {
        var func = "";
        var resp = "";
        var indent = "";
        var addto = "code";
    }

    var msgPackage = srvtypeRaw.split("/")[0];
    var msgtype = srvtypeRaw.split("/")[1];

    var importCode = [importSyntax(msgPackage + ".srv", msgtype), importSyntax(msgPackage + ".srv", msgtype + "Request")];
    importCode = importCode.concat(importExtraPackages()).join("");


    var varInits = [];
    for (var inCnt = 2; inCnt <= block.args0.length; inCnt++) {
        varInits.push("var value_var" + inCnt.toString() + "=Blockly.Python.valueToCode(block,\'var" + inCnt.toString() + "\', Blockly.Python.ORDER_ATOMIC);");
    }
    varInits.push("Blockly.Python.InitROS();");

    var msgNames = Array.prototype.map.call(srvFields, elem => elem.value);
    var msgValues = Array.prototype.map.call(fieldValues, elem => elem.value);
    var codeFlags = Array.prototype.map.call(srvChBoxes, elem => elem.checked);

    var blocklyCode = [];

    for (i = 0; i < msgNames.length; i++) {
        var name = msgNames[i];
        var isCode = codeFlags[i];
        var value = replaceDollars(msgValues[i].replace(/\n/g, "\\n" + indent).replace(/\'/g, "\\'").replace(/\"/g, "\\'"), inCnt);
        if (value == "") return ""

        if (isCode) {
            blocklyCode.push(value + "\\n");
        } else {
            blocklyCode.push(name + "=" + value + "\\n");
        }

    }
    blocklyCode.push("reqparams=(" + msgNames.join(",") + ")\\n");

    var message = blocklyCode.map(v => addto + "+='" + indent + v + "'").join(";");

    var MsgImport = "Blockly.Python.definitions_['rospy_init_node']+='\\nHobbbitLib.importMsg(\\'" + msgPackage + ".srv\\',\\'" + msgtype + "\\')\\n'";

    var pubCode = addto + "+=" + resp + "Blockly.Python.NodeName+'.callService(\\'" + service + "\\', \\'" + msgtype + "\\', reqparams)\\n';"

    message = useResponse ? ["var func=''", message, pubCode, func + "+func"].join(";") : message;
    pubCode = useResponse ? "code+='srv" + blockId + "()';" : pubCode;

    var pyCode = [message, MsgImport, pubCode].join(";");
    var codeinit = useResponse ? "var code='';" : "var code='\\n';";

    return [varInits.join(""), importCode, codeinit, pyCode, "return [code, Blockly.Python.ORDER_NONE]"].join("")
}

function getCallbacks(blockId) {
    var defs = [];
    var assigs = [];
    cbParams = {
        donecb: "(status,result)",
        activecb: "()",
        feedbackcb: "(feedback)"

    }

    for (var shape of ["donecb", "activecb", "feedbackcb"]) {
        var cb = document.getElementById(shape).value;
        var cbId = shape + "_" + blockId;
        if (cb && cb != "") {
            cb = cb.replace(/\n/g, "\\n").replace(/\t/g, "\\t");
            cb = cb.split("\\n").map(v => "\\t" + v).join("\\n");
            var cbTemp = "Blockly.Python.definitions_[\'%" + cbId + "\'] = \'def " + cbId + cbParams[shape] + ":\\n" + cb + "\';";
            defs.push(cbTemp);
            assigs.push("," + shape.slice(0, -2) + "_cb" + "=" + cbId);
        }
    }

    return { definition: defs.join(""), assignment: assigs.join("") };
}

function getCodeAction(block, blockId) {
    var serverName = document.getElementById("action").value;
    var msgtypeRaw = document.getElementById("actiontype").value;
    var message = document.getElementById("goal").value;
    var timeout = parseFloat(document.getElementById("actiontimeout").value);
    timeout = (timeout % 1 == 0 ? timeout + ".0" : timeout).toString();

    var cbDefinitions = getCallbacks(blockId).definition;
    var cbAssignment = getCallbacks(blockId).assignment;

    var msgPackage = msgtypeRaw.split("/")[0];
    var msgtype = msgtypeRaw.split("/")[1];

    var importCode = [importSyntax(msgPackage + ".msg", msgtype)];
    importCode = importCode.concat(importExtraPackages()).join("");

    var varInits = [];
    for (var inCnt = 2; inCnt <= block.args0.length; inCnt++) {
        varInits.push("var value_var" + inCnt.toString() + "=Blockly.Python.valueToCode(block,\'var" + inCnt.toString() + "\', Blockly.Python.ORDER_ATOMIC);");
    }
    varInits.push("Blockly.Python.InitROS();");


    var re = /\$([0-9]+)\$/gm;
    while ((match = re.exec(message)) != null) {
        if (inCnt < parseInt(match[1]) + 2) {
            alert("Only " + (inCnt - 2).toString() + " inputs defined, but more used in message!")
            return ""
        }
        message = message.replace(match[0], "'+value_var" + (parseInt(match[1]) + 1).toString() + "+'");
    }

    message = "var code='\\n';" + message.split("\n").map(e => "code+=\'" + e + "\\n\'").join(";");
    var MsgImport = "code+='Hobbitlib.importMsg(\\'" + msgPackage + ".msg\\',\\'" + msgtype + "\\')\\n'";

    var clientCode = ["client = actionlib.SimpleActionClient(\\'" + serverName + "\\', " + msgtype + ")", "client.wait_for_server()", "client.send_goal(goal" + cbAssignment + ")", "client.wait_for_result(rospy.Duration.from_sec(" + timeout + "))"];

    clientCode = clientCode.map(v => "code+='" + v + "\\n'").join(";");

    var pyCode = [message, MsgImport, clientCode].join(";") + ";";

    return [varInits.join(""), importCode, cbDefinitions, pyCode, "return code"].join("")
}

function create(mode) {
    var block = getBlockDefintion();
    var blockId = mode == "create" ? uniqueId() : currentBlock.id;

    if (selectedType == "Topic") {
        var code = getCodeTopic(block);
        var metaInfo = getMetaInfoTopic();
    } else if (selectedType == "Service") {
        var code = getCodeService(block, blockId);
        var metaInfo = getMetaInfoService();
    } else if (selectedType == "Action") {
        var code = getCodeAction(block, blockId);
        var metaInfo = getMetaInfoAction();
    } else return
    
    if (code && code != "") {

        var newBlock = { "id": blockId, "meta": JSON.stringify(metaInfo), "code": code, "block": JSON.stringify(block) };

        if (mode == "create") {
            $.post('/block/create',
                {
                    block: newBlock
                }, function (data, status) {
                    if (status == "success") {
                        window.alert("Block successfully created");
                        window.location.href = window.location.origin + window.location.pathname + "?" + newBlock.id
                    } else {
                        alert("Something went wrong!");
                    }
                });
        } else {
            $.put('/block/update',
                {
                    block: newBlock
                }, function (data, status) {
                    if (status == "success") {
                        newBlock.meta = JSON.parse(newBlock.meta);
                        currentBlock = newBlock;
                        setFormValues(currentBlock.meta);
                        toggleEditMode(editMode);
                        window.alert("Block successfully updated.");
                    } else {
                        alert("Something went wrong!");
                    }
                });
        }
    }
}

function deleteBlock() {
    if (confirm("Do you really want to delete this block?") == true) {
        $.delete("/block/delete/" + currentBlock.id,
            {}, function (data, status) {
                if (status == "success") {
                    window.alert("Block successfully deleted.");
                    clearform();
                } else {
                    alert("Something went wrong!");
                }
            });
    }
}

function addInput(value) {
    inputCnt += 1;
    var x = document.createElement("INPUT");
    x.setAttribute("placeholder", "Name");
    x.setAttribute("id", "in" + inputCnt);
    x.setAttribute("type", "text");
    x.setAttribute("class", "validate");
    if (value) x.setAttribute("value", value);
    document.getElementById("inputs").appendChild(x);
}

function removeInput() {
    var inputsDiv = document.getElementById("inputs")
    var inputs = document.getElementById("inputs").getElementsByTagName("INPUT");

    if (inputCnt > 0) {
        inputsDiv.removeChild(inputs[inputs.length - 1]);
        inputCnt -= 1;
    }
}

function addSrvField(name, value, isCode) {
    srvFieldCnt += 1;

    var wrap = document.createElement("DIV");
    wrap.setAttribute("id", "srvMsg" + srvFieldCnt);

    var colps = document.createElement("DIV");
    colps.setAttribute("class", "blockly-collapsible");

    var inp = document.createElement("INPUT");
    inp.setAttribute("placeholder", "Name");
    inp.setAttribute("id", "srvField" + srvFieldCnt);
    inp.setAttribute("type", "text");
    inp.setAttribute("class", "validate srvField");
    if (name) inp.setAttribute("value", name);
    colps.appendChild(inp);

    var colCont = document.createElement("DIV");
    colCont.setAttribute("class", "col-content");

    var srvVal = document.createElement("TEXTAREA");
    srvVal.setAttribute("id", "srvMsgVal" + srvFieldCnt.toString());
    srvVal.setAttribute("placeholder", "Enter value...");
    srvVal.setAttribute("style", 'height:50px; max-width: 100%; min-width: 100%;');
    if (value) srvVal.value = value;

    var chBoxLbl = document.createElement("LABEL");
    var chBox = document.createElement("INPUT");
    chBox.setAttribute("id", "srvCode" + srvFieldCnt);
    chBox.setAttribute("type", "checkbox");
    chBox.setAttribute("class", "filled-in srvCode");
    if (isCode) chBox.checked = isCode;
    var chBoxDesc = document.createElement("SPAN");
    chBoxDesc.innerHTML = "Code";

    chBoxLbl.appendChild(chBox);
    chBoxLbl.appendChild(chBoxDesc);

    colCont.appendChild(srvVal);
    colCont.appendChild(chBoxLbl);

    wrap.appendChild(colps);
    wrap.appendChild(colCont);

    document.getElementById("requestFields").appendChild(wrap);
    updateListener()
}

function removeSrvField() {
    var fieldsDiv = document.getElementById("requestFields");
    var srvField = document.getElementById("srvMsg" + srvFieldCnt);
    if (srvFieldCnt > 0) {
        fieldsDiv.removeChild(srvField);
        srvFieldCnt -= 1;
    }
}