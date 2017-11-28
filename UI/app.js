const path = './demos/';
const fs = require('fs');
const { execSync } = require('child_process');
var qs = require('querystring');
// var copyFile = require('quickly-copy-file');
var express = require('express');
var app = express();
// const PORT = 3000;
const PORT = Number(process.argv.slice(2));
// const LIBFILE = 'HobbitLib';

function loadIndex(req, res) {
	res.sendFile('./index.html', { root: __dirname });
};

function runCode(req, res) {
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
		// fs.writeFile('/home/demo/catkin_ws/src/hokuyo_node/test/'+post.filename, post.code, (err) => { 
		fs.writeFile('../../src/'+post.filename, post.code, (err) => {
			if (err) throw err;

			console.log('Demo has been started!');
		});
		
		// copyFile(LIBFILE, '../../src/'+LIBFILE, function(error) {
			// if (error) return console.error(error);
		// });
		
		// exec('bash run.sh', (err, stdout, stderr) => {
			  // if (err) {
				// console.error(err);
				// return;
			  // }
			// });
		console.log('start');
		execSync('start test.bat', (err, stdout, stderr) => {
			  if (err) {
				console.error(err);
				return;
			  }
			});
		console.log('end');
			
		
	});
	
	res.statusCode = 200;
	res.end();
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
  console.log('Hobbit blockly is now running at port '+PORT+'!');
});