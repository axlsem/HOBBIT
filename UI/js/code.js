/* Functions */
function show_blockly() {
	document.getElementById("blocklyDiv").style.display = 'block';
	document.getElementById("blocklyArea").style.display = 'block';
	document.getElementById("nav-home").setAttribute("class", "current");

	window.location.reload(true);
	// alert("resizing");
	// window.dispatchEvent(new Event('resize'));
  };
  
  function hide_blockly() {
      document.getElementById("blocklyDiv").style.display = 'none';
      document.getElementById("blocklyArea").style.display = 'none';
      document.getElementById("editor").style.display = 'block';
      document.getElementById("nav-code").setAttribute("class", "current");
      document.getElementById("nav-home").removeAttribute("class");

      // onresize();
      window.dispatchEvent(new Event('resize'));

      // show the code
      Blockly.Python.addReservedWords('code');
      var code = Blockly.Python.workspaceToCode(workspace);

      var editor = ace.edit("editor");
      editor.setTheme("ace/theme/chrome");
      editor.getSession().setMode("ace/mode/python");
      editor.getSession().setUseWrapMode(true);
      editor.setShowPrintMargin(false);
      editor.setValue(code);
  };
  
  function run_code() {
	var pycode = Blockly.Python.workspaceToCode(workspace);

	if (confirm("Do you really want to run the demo?") == true) {
		$.post("/run",
			{
			  code: pycode,
			  filename: "run.py"
			},function(data, status){
				if (status=="success") {
					alert("Demo is now running!");
				} else {
					alert("Something went wrong!");
				}
		});
	}
	};
	
	function save_demo() {
		var filename = prompt("Please enter demo name", "Demo");
		
		if (filename == null || filename == "") {
			console.log('Cancelled')
		} else {
			
			var xml = Blockly.Xml.workspaceToDom(workspace);
			var xml_text = Blockly.Xml.domToText(xml);
			$.post("/save",
				{
				  content: xml_text,
				  filename: filename+'.xml',
				  overwrite: false
				},function(data, status){
					if (status=="success") {
						if (data.result == true) {
							alert("Demo saved.");
						} else {
							if (confirm("File already exists. Do you want overwrite it?") == true) {
								$.post ("/save",
									{
										content: xml_text,
										filename: filename+'.xml',
										overwrite: true
									});
							}
						}
					} else {
						alert("Oops...Something went wrong!");
					}
				});
		}
    };
	
	function load_demo() {
		var can_load_file = false;
		if (workspace.getAllBlocks().length > 0) {
			can_load_file = confirm("Current workspace is not empty. Do you want to override it?");
		} else {
			can_load_file = true;
		}
		
		<!-- if (can_load_file == true) { -->
			<!-- var filename = prompt("Please enter demo name", "Demo"); -->

			<!-- if (filename == null || filename == "") { -->
				<!-- console.log('Cancelled') -->
			<!-- } else { -->
				<!-- $.post("/load", -->
					<!-- { -->
					  <!-- filename: filename+'.xml' -->
					<!-- },function(data, status){ -->
						<!-- if (status=="success") { -->
							<!-- workspace.clear(); -->
							<!-- var xml = Blockly.Xml.textToDom(data); -->
							<!-- Blockly.Xml.domToWorkspace(xml, workspace); -->
							<!-- console.log(workspace); -->

						<!-- } else { -->
							<!-- alert("Something went wrong!"); -->
						<!-- } -->
				<!-- }); -->
			<!-- } -->
		<!-- } -->

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
		console.log("Workspace cleaned.");
	}
    };

/* Buttons */
var logo = document.getElementById("logo");
var home_btn = document.getElementById("nav-home");
var run_btn = document.getElementById("nav-run");
var code_btn = document.getElementById("nav-code");
var save_btn = document.getElementById("nav-save");
var load_btn = document.getElementById("nav-load");
var clear_btn = document.getElementById("nav-clear");

logo.onclick = show_blockly;
home_btn.onclick = show_blockly;
run_btn.onclick = run_code;
code_btn.onclick = hide_blockly;
save_btn.onclick = save_demo;
load_btn.onclick = load_demo;
clear_btn.onclick = clear_ws;

/* Blockly */
var blocklyArea = document.getElementById('blocklyArea');
  var blocklyDiv = document.getElementById('blocklyDiv');
  var workspace = Blockly.inject(blocklyDiv,
    {toolbox: document.getElementById('toolbox'),
       scrollbars: true,
	   sounds: false,
       rtl: false,
       zoom:
           {enabled: true,
            controls: true,
            wheel: true,
            maxScale: 4,
            minScale: .25,
            scaleSpeed: 1.1
           },
       grid:
           {spacing: 25,
            length: 3,
            colour: '#ccc',
            snap: true},
       trashcan: true});
	   
  var onresize = function(e) {
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
    blocklyDiv.style.height = blocklyArea.offsetHeight-80 + 'px';
  };
  window.addEventListener('resize', onresize, false);
  onresize();
  Blockly.svgResize(workspace);
  

/* ACE editor */
document.getElementById('editor').style.fontSize='16px';