/* Functions */
function toIndex() {
	window.location.href = window.location.origin;
};

function reload_blockly() {
	document.getElementById("blocklyDiv").style.display = 'block';
	document.getElementById("blocklyArea").style.display = 'block';
	document.getElementById("nav-home").setAttribute("class", "current");
	document.getElementById("nav-home").parentElement.setAttribute("class", "border-current");

	localstorage();

	window.location.reload(true);
};

function show_blockly() {
	var blocklyCode = Blockly.Python.workspaceToCode(workspace);
	var editorCode = ace.edit("editor").getValue();
	if (blocklyCode == editorCode) {
		reload_blockly();
	} else {
		if (confirm("Edited code will be lost. Do you want to continue?")) {
			reload_blockly();
		}
	}

};

function hide_blockly() {
	home_btn.onclick = show_blockly;
	document.getElementById("blocklyDiv").style.display = 'none';
	document.getElementById("blocklyArea").style.display = 'none';
	document.getElementById("editor").style.display = 'block';
	document.getElementById("nav-code").setAttribute("class", "current");
	document.getElementById("nav-code").parentElement.setAttribute("class", "border-current");
	document.getElementById("nav-home").removeAttribute("class");
	document.getElementById("nav-home").parentElement.removeAttribute("class");
	document.getElementById("icon-blocks").setAttribute("class", "icon-blocks-passive");
	document.getElementById("icon-code").setAttribute("class", "icon-code-current");

	// onresize();
	window.dispatchEvent(new Event('resize'));

	// show the code
	Blockly.Python.addReservedWords('code');
	var code = getCode();

	var editor = ace.edit("editor");
	editor.setTheme("ace/theme/chrome");
	editor.getSession().setMode("ace/mode/python");
	editor.getSession().setUseWrapMode(true);
	editor.setShowPrintMargin(false);
	editor.setValue(code);
};

function getCode() {
	var blocklyCode = Blockly.Python.workspaceToCode(workspace);
	var editorCode = ace.edit("editor").getValue();
	return editorCode || blocklyCode
}

function run_code() {
	var pycode = getCode();

	if (confirm("Do you really want to run the demo?") == true) {
		$.post("/run",
			{
				code: pycode,
				filename: "run.py",
				main: false
			}, function (data, status) {
				if (status == "success") {
					alert(data.result);
				} else {
					alert("Something went wrong!");
				}
			});
	}
};

function save_demo_locally() {
	var filename = 'Demo.xml';
	var xml = Blockly.Xml.workspaceToDom(workspace);
	var xml_text = Blockly.Xml.domToText(xml);
	var blob = new Blob([xml_text], { type: 'text/xml' });
	if (window.navigator.msSaveOrOpenBlob) {
		window.navigator.msSaveBlob(blob, filename);
	} else {
		var elem = window.document.createElement('a');
		elem.href = window.URL.createObjectURL(blob);
		elem.download = filename;
		document.body.appendChild(elem);
		elem.click();
		document.body.removeChild(elem);
	}
}

function save_demo_robot() {
	var demoname = prompt("Please enter demo name", "");

	if (typeof demoname == "string") {
		var pycode = getCode();
		var hasBlanks = demoname.indexOf(" ") >= 0;

		if (demoname == "" || hasBlanks) {
			alert("Enter a valid name.");
			console.log('Cancelled saving of \"' + demoname + "\"");
		} else {

			var xml = Blockly.Xml.workspaceToDom(workspace);
			var xml_text = Blockly.Xml.domToText(xml);
			$.post("/save",
				{
					content: xml_text,
					demoname: demoname,
					overwrite: false,
					code: pycode
				}, function (data, status) {
					if (status == "success") {
						if (data.result == true) {
							alert("Demo saved.");
						} else {
							if (confirm("File already exists. Do you want overwrite it?") == true) {
								$.post("/save",
									{
										content: xml_text,
										demoname: demoname,
										overwrite: true,
										code: pycode
									});
							}
						}
					} else {
						alert("Oops...Something went wrong!");
					}
				});
		}
	}
};

function list_demofiles(files) {
	var demolist = document.getElementById("list-demofiles");

	$("#list-demofiles").empty();

	var el = document.createElement("option");
	el.textContent = "Select Demo";
	el.value = i;
	demolist.appendChild(el);

	for (var i = 0; i < files.length; i++) {
		var opt = files[i].slice(0, -4);
		var el = document.createElement("option");
		el.textContent = opt;
		el.value = i + 1;
		demolist.appendChild(el);
	};

};

function show_demolist() {
	var hobbit_modal = document.getElementById("load-hobbit-modal");
	hobbit_modal.style.display = 'block';

	$.post("/demolist",
		{
		}, function (data, status) {
			if (status == "success") {
				list_demofiles(data.result);

			} else {
				alert("Something went wrong!");
			}
		});
};

function load_demo_robot() {
	var can_load_file = false;

	var demolist = document.getElementById("list-demofiles");
	var demo_selected = demolist.options.selectedIndex;

	if (demo_selected == 0) {
		alert("Select a demo!");
		return;
	}
	else {
		var filename = demolist.options[demo_selected].text;
	}
	if (workspace.getAllBlocks().length > 0) {
		can_load_file = confirm("Current workspace is not empty. Do you want to override it?");
	} else {
		can_load_file = true;
	}

	if (can_load_file == true) {
		if (filename == null || filename == "") {
			console.log('Cancelled')
		} else {
			$.post("/load",
				{
					filename: filename + '.xml'
				}, function (data, status) {
					if (status == "success") {
						workspace.clear();
						var xml = Blockly.Xml.textToDom(data.result);
						Blockly.Xml.domToWorkspace(xml, workspace);
						console.log("Loading workspace from Hobbit.");
						close_load_modal();

					} else {
						alert("Something went wrong!");
					}
				});
		}
	}
};

function load_demo_locally() {
	var can_load_file = false;
	if (workspace.getAllBlocks().length > 0) {
		can_load_file = confirm("Current workspace is not empty. Do you want to override it?");
	} else {
		can_load_file = true;
	}


	if (true == can_load_file) {
		var input_field_name = 'load_workspace_from_file_input';
		var file_input = document.getElementById(input_field_name);
		if (null == file_input) {
			file_input = document.createElement('input');
			file_input.type = 'file';
			file_input.id = input_field_name;
			file_input.name = input_field_name;
			file_input.addEventListener('change',
				function (evt) {
					var files = evt.target.files;
					if (files.length > 0) {
						var file = files[0];
						var reader = new FileReader();
						reader.onload = function () {
							workspace.clear();
							var xml = Blockly.Xml.textToDom(this.result);
							console.log("Loading workspace from file.");
							Blockly.Xml.domToWorkspace(workspace, xml);
						};
						reader.readAsText(file);
						// This is done in order to allow open the same file several times in the row
						document.body.removeChild(file_input);
					}
				}, false);
			// Hidding element from view
			file_input.style = 'position: fixed; top: -100em';
			document.body.appendChild(file_input);
		}
		file_input.click();
	}
};

function clear_ws() {
	if (confirm("Do you really want to clear workspace?") == true) {
		workspace.clear();
		localStorage.removeItem("blocks_cache")
		console.log("Workspace cleaned.");
	}
};

function close_load_modal() {
	var modal = document.getElementById("load-hobbit-modal");
	modal.style.display = "none";
}

/* Buttons */
var logo = document.getElementById("logo");
var home_btn = document.getElementById("nav-home");
var run_btn = document.getElementById("nav-run");
var code_btn = document.getElementById("nav-code");
var save_robot_btn = document.getElementById("nav-save-robot");
var save_locally_btn = document.getElementById("nav-save-locally");
var show_demolist_btn = document.getElementById("nav-show-demolist");
var load_robot_btn = document.getElementById("nav-load-robot");
var load_locally_btn = document.getElementById("nav-load-locally");
var clear_btn = document.getElementById("nav-clear");
var modal_close_btn = document.getElementsByClassName("close")[0];

logo.onclick = toIndex;
// home_btn.onclick = show_blockly;
run_btn.onclick = run_code;
code_btn.onclick = hide_blockly;
save_robot_btn.onclick = save_demo_robot;
save_locally_btn.onclick = save_demo_locally;
show_demolist_btn.onclick = show_demolist;
load_robot_btn.onclick = load_demo_robot;
load_locally_btn.onclick = load_demo_locally;
clear_btn.onclick = clear_ws;
modal_close_btn.onclick = close_load_modal;

var load_hobbit_modal = document.getElementById("load-hobbit-modal");
window.onclick = function (event) {
	if (event.target == load_hobbit_modal) {
		load_hobbit_modal.style.display = "none";
	}
}

/* Blockly */
var blocklyArea = document.getElementById('blocklyArea');
var blocklyDiv = document.getElementById('blocklyDiv');
var workspace = Blockly.inject(blocklyDiv,
	{
		toolbox: document.getElementById('toolbox'),
		scrollbars: true,
		sounds: false,
		rtl: false,
		media: '../img/media/',
		zoom:
		{
			enabled: true,
			controls: true,
			wheel: true,
			maxScale: 4,
			minScale: .25,
			scaleSpeed: 1.1
		},
		grid:
		{
			spacing: 25,
			length: 3,
			colour: '#ccc',
			snap: true
		},
		trashcan: true
	});

$.get("/toolbox", function (data, status) {
	if (status == "success") {

		for (var custBlock of data.blocks) {
			var block = custBlock.block;
			var name = custBlock.name;
			var code = custBlock.code;

			
			eval("Blockly.Blocks."+name+" = {init:function(){this.jsonInit("+block+")}}");
			eval("Blockly.Python."+name+"=function(block){"+code+"}");
			
		}
		
		var parser = new DOMParser();
		var xmlToolbox = parser.parseFromString(data.toolbox,"text/xml");
		workspace.updateToolbox(xmlToolbox.getElementById("toolbox"));

		// var testblock = '';
		// var tescode = "";
		
		// eval("Blockly.Blocks.newblock = {init:function(){this.jsonInit("+testblock+")}}");
		// eval("Blockly.Python.newblock=function(block){"+tescode+"}");

		// var block0 = workspace.newBlock("newblock");
		// block0.initSvg();
		// block0.render();
		
		// var block1 = workspace.newBlock("custom1");
		// block1.initSvg();
		// block1.render();

		// var parentConnection = block0.getInput('custom1').connection;
		// var childConnection = block1.previousConnection;
		// parentConnection.connect(childConnection);

	} else {
		alert("Something went wrong!");
	}
});

var onresize = function (e) {
	// Compute the absolute coordinates and dimensions of blocklyArea.
	var element = blocklyArea;
	var x = 0;
	var y = 0;
	do {
		x += element.offsetLeft;
		y += element.offsetTop;
		element = element.offsetParent;
	} while (element);
	// Position blocklyDiv over blocklyArea.
	blocklyDiv.style.left = x + 'px';
	blocklyDiv.style.top = y + 'px';
	blocklyDiv.style.width = blocklyArea.offsetWidth + 'px';
	blocklyDiv.style.height = blocklyArea.offsetHeight - 80 + 'px';
};
window.addEventListener('resize', onresize, false);
onresize();
Blockly.svgResize(workspace);

var blocklyAreaHeightAct = $("#blocklyArea").height();
$("#blocklyArea").height(blocklyAreaHeightAct - 80);

/* ACE editor */
document.getElementById('editor').style.fontSize = '16px';
var editorHeightAct = $("#editor").height();
$("#editor").height(editorHeightAct - 80);

/* Reload */
function restorelocal() {
	var xml_text = localStorage.getItem("blocks_cache");
	try {
		var xml = Blockly.Xml.textToDom(xml_text);
		Blockly.Xml.domToWorkspace(workspace, xml);

		// automate_localstorage();
	}
	catch (err) {
		console.log(err);
		// automate_localstorage();
	}
}

function automate_localstorage() {
	localstorage();
	setTimeout(automate_localstorage, 1000);
}

// Save stuff on local storage
function localstorage() {
	var xml = Blockly.Xml.workspaceToDom(workspace);
	var xml_text = Blockly.Xml.domToText(xml);
	localStorage.setItem("blocks_cache", xml_text);
}