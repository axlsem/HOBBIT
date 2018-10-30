$('.hide-on-study').hide();
$('.show-on-study').show();
$("#logo").prop("onclick", null).off("click");

var starttime;
var endtime;
var lastaction;
var ts = [];

$("#nav-start").click(function () {
    $('#blocklyDiv').show();
    starttime = Date.now();
    lastaction = starttime;
    ts.push(starttime);
    $('#nav-start-wrapper').hide();
    $('#nav-stop').show();


});

$('#nav-stop').click(function () {
    endtime = Date.now();
    $('#blocklyDiv').hide();
    $('#nav-stop-wrapper').hide();
    $('#nav-submit').show();
})

$('#nav-submit').click(function () {
    var xml = Blockly.Xml.workspaceToDom(workspace);
    var xml_text = Blockly.Xml.domToText(xml);
    localStorage.setItem('study_blockly',xml_text);

    var userId = prompt("Please enter user ID", "");

    if (typeof userId == "string") {

        var data = { user: userId, times: ts, time: endtime - starttime, code: xml_text, type: "blockly" };

        $.post("/submit/" + userId,
            {
                data: JSON.stringify(data)
            }, function (data, status) {
                if (status == "success") {
                    window.alert("Thank you");
                } else {
                    alert("Something went wrong!");
                }
            });
    }

})

function trackCahnges(event) {
    var changeTypes = [Blockly.Events.BLOCK_DELETE, Blockly.Events.BLOCK_CREATE, Blockly.Events.BLOCK_CHANGE, Blockly.Events.VAR_CREATE, Blockly.Events.VAR_DELETE, Blockly.Events.VAR_RENAME];

    if (changeTypes.indexOf(event.type) >= 0) {
        var tresh = 10 * 1000;
        var t = Date.now();
        if (t - lastaction > tresh) {
            ts.push(t);
            console.log(t);
        }
        lastaction = t;
    }
}
workspace.addChangeListener(trackCahnges);