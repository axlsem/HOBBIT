// Generate code from workspace
var code = Blockly.Python.workspaceToCode(workspace);
// Create Ace instance and set preferences
var editor = ace.edit("editor");
editor.setTheme("ace/theme/chrome");
editor.getSession().setMode("ace/mode/python");
editor.getSession().setUseWrapMode(true);
editor.setShowPrintMargin(false);
// Display code
editor.setValue(code);