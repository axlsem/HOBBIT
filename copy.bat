@echo off

xcopy D:\_Diplomarbeit\HOBBIT\blockly\blockly_accessible_compressed.js D:\_Diplomarbeit\HOBBIT\dev /Y
xcopy D:\_Diplomarbeit\HOBBIT\blockly\blockly_accessible_uncompressed.js D:\_Diplomarbeit\HOBBIT\dev /Y
xcopy D:\_Diplomarbeit\HOBBIT\blockly\blockly_compressed.js D:\_Diplomarbeit\HOBBIT\dev /Y
xcopy D:\_Diplomarbeit\HOBBIT\blockly\blockly_uncompressed.js D:\_Diplomarbeit\HOBBIT\dev /Y
xcopy D:\_Diplomarbeit\HOBBIT\blockly\blocks_compressed.js D:\_Diplomarbeit\HOBBIT\dev /Y
xcopy D:\_Diplomarbeit\HOBBIT\blockly\python_compressed.js D:\_Diplomarbeit\HOBBIT\dev /Y
xcopy D:\_Diplomarbeit\HOBBIT\blockly\blocks\hobbit.js D:\_Diplomarbeit\HOBBIT\dev\blocks\hobbit.js /Y
xcopy D:\_Diplomarbeit\HOBBIT\blockly\core D:\_Diplomarbeit\HOBBIT\dev\core /Y
xcopy D:\_Diplomarbeit\HOBBIT\blockly\msg D:\_Diplomarbeit\HOBBIT\dev\msg /Y /S
REM xcopy D:\_Diplomarbeit\HOBBIT\blockly\generators D:\_Diplomarbeit\HOBBIT\dev\generators /Y
xcopy D:\_Diplomarbeit\HOBBIT\blockly\generators\python.js D:\_Diplomarbeit\HOBBIT\dev\generators\ /Y
xcopy D:\_Diplomarbeit\HOBBIT\blockly\generators\python D:\_Diplomarbeit\HOBBIT\dev\generators\python /Y
xcopy D:\_Diplomarbeit\HOBBIT\blockly\blockly_compressed.js D:\_Diplomarbeit\HOBBIT\UI\js\blockly_compressed.js /Y
xcopy D:\_Diplomarbeit\HOBBIT\blockly\blocks_compressed.js D:\_Diplomarbeit\HOBBIT\UI\js\blocks_compressed.js /Y
xcopy D:\_Diplomarbeit\HOBBIT\blockly\python_compressed.js D:\_Diplomarbeit\HOBBIT\UI\js\python_compressed.js /Y