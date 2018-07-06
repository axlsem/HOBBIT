const path = './demos/';
const pathXML = './demos/xml/';
const pathPy = './demos/src/';
const fs = require('fs');
const { exec } = require('child_process');
const PORT = process.env.port || 3000;
// const PORT = Number(process.argv.slice(2));

var qs = require('querystring');
var express = require('express');
var app = express();
var locked = false;
var param = String(process.argv.slice(2));

var isDev = param == 'dev';

function loadIndex(req, res) {
	res.sendFile('./index.html', { root: __dirname });
};

function loadBlockly(req, res) {
	res.sendFile('./blockly.html', { root: __dirname });
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
			var destpath = '../../src/';
			
			var isMain = post.main;

			if (isDev) {
				var cmd = 'start test.bat';
			} else {
				var cmd = 'bash run.sh';
			}
			
			if (isMain == 'true') {
				fs.copyFile(post.sourcepath + post.sourcefile, destpath + post.filename, (err) => {
					if (err) throw err;
				});
			} else {
				fs.writeFile(destpath + post.filename, post.code, (err) => {
					if (err) throw err;
				});
			}

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
		res.status(200).send({ "result": feedback });
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
		var filenameXML = post.demoname + '.xml';
		var filenamePy = post.demoname + '.py';

		if (fs.existsSync(pathXML + filenameXML)) {
			file_exists = true;
		}

		var can_be_saved = (file_exists == false) || (post.overwrite == 'true');

		if (can_be_saved == true) {
			fs.writeFile(pathXML + filenameXML, post.content, (err) => {
				if (err) throw err;
				console.log('Demo saved!');
			});

			fs.writeFile(pathPy + filenamePy, post.code, (err) => {
				if (err) throw err;
				console.log('Demo saved!');
			});
		}

		res.status(200).send({ "result": can_be_saved });
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

		fs.readFile(pathXML + post.filename, 'utf8', function (err, data) {
			if (err) throw err;
			res.status(200).send({ "result": data });
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

		data = fs.readdirSync(pathXML);
		res.status(200).send({ "result": data });
	});

}

function deleteDemo(req, res) {
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

		fs.unlink(pathXML + post.demoname + '.xml', (err) => {
			if (err) throw err;
		});

		fs.unlink(pathPy + post.demoname + '.py', (err) => {
			if (err) throw err;
		});

		res.status(200).send({ "result": data });
	});
}

app.use(express.static(__dirname + '/'));

app.get('/', loadIndex);
app.get('/blockly', loadBlockly);
app.post('/run', runCode);
app.post('/save', saveWorkspace);
app.post('/load', loadDemo);
app.post('/demolist', showDemolist);
app.post('/delete', deleteDemo);

app.listen(PORT, '0.0.0.0', function () {
	console.log('Hobbit blockly is now listening to port ' + PORT + '!');
});