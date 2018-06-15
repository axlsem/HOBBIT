const path = './demos/';
const fs = require('fs');
const { exec } = require('child_process');
const PORT = process.env.port || 3000;
// const PORT = Number(process.argv.slice(2));

var qs = require('querystring');
var express = require('express');
var app = express();
var locked = false;
var cmd = '';
var param = String(process.argv.slice(2));

if (param == 'dev') {
	cmd = 'start test.bat';
} else {
	cmd = 'bash run.sh';
}
// var copyFile = require('quickly-copy-file');
// const LIBFILE = 'HobbitLib';

function loadIndex(req, res) {
	res.sendFile('./index.html', { root: __dirname });
};

function runCode(req, res) {
	var body = '';
	var feedback = '';
	
	req.on('error', function (err) {
		console.error(err);
	});
	
	req.on('data', function (data) {
		body += data;

		if (body.length > 1e6)
			req.connection.destroy();
	});

	req.on('end', function () {
		if (!locked) {
			locked = true;
			feedback = "Demo is now running!";
			var post = qs.parse(body);
			// fs.writeFile('/home/demo/catkin_ws/src/hokuyo_node/test/'+post.filename, post.code, (err) => { 
			fs.writeFile('../../src/'+post.filename, post.code, (err) => {
				if (err) throw err;
			});
			
			// copyFile(LIBFILE, '../../src/'+LIBFILE, function(error) {
				// if (error) return console.error(error);
			// });

			exec(cmd, (err, stdout, stderr) => {
					locked = false;
					if (err) {
					console.error(err);
					return;
				  }
				});
		} else {
			feedback = "Another demo is already running! Please try again later.";
		}
	res.status(200).send({"result": feedback});
	});
};

function saveWorkspace(req, res) {
	var body = '';
	
	req.on('error', function (err) {
		console.error(err);
	});
	
	req.on('data', function (data) {
		body += data;

		if (body.length > 1e6)
			req.connection.destroy();
	});

	req.on('end', function () {
		var post = qs.parse(body);
		var file_exists = false;
		
		if (fs.existsSync(path+post.filename)) {
			file_exists = true;
		}
		
		var can_be_saved = (file_exists == false) || (post.overwrite == 'true');
		
		if (can_be_saved == true) {
			fs.writeFile('./demos/'+post.filename, post.content, (err) => {
				if (err) throw err;

				console.log('Demo saved!');
			});
		}
		
		res.status(200).send({"result": can_be_saved});
	});
	
};

function loadDemo(req, res) {
	var body = '';
	
	req.on('error', function (err) {
		console.error(err);
	});
	
	req.on('data', function (data) {
		body += data;

		if (body.length > 1e6)
			req.connection.destroy();
	});

	req.on('end', function () {
		var post = qs.parse(body);
		
		fs.readFile('./demos/'+post.filename, 'utf8', function (err,data) {
			if (err) throw err;
			res.status(200).send({"result": data});
		});
	});
	
}

function showDemolist(req, res) {
	var body = '';
	
	req.on('error', function (err) {
		console.error(err);
	});
	
	req.on('data', function (data) {
		body += data;

		if (body.length > 1e6)
			req.connection.destroy();
	});

	req.on('end', function () {
		var post = qs.parse(body);
		
		data = fs.readdirSync('./demos/');
		res.status(200).send({"result": data});
	});
	
}

app.use(express.static(__dirname + '/'));

app.get('/', loadIndex);
app.post('/run', runCode);
app.post('/save', saveWorkspace);
app.post('/load', loadDemo);
app.post('/demolist', showDemolist);

app.listen(PORT,'0.0.0.0', function () {
  console.log('Hobbit blockly is now listening to port '+PORT+'!');
});