const path = './demos/';
const fs = require('fs');
const { exec } = require('child_process');
var qs = require('querystring');
var express = require('express');
var app = express();
const PORT = 8080;

app.use(express.static(__dirname + '/'));

app.get('/', function (req, res) {
	res.sendFile('./index.html', { root: __dirname });
});

app.post('/run', function (req, res) {
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
		fs.writeFile(post.filename, post.code, (err) => {  
			if (err) throw err;

			console.log('Demo has been started!');
		});
		// exec('run.sh', (err, stdout, stderr) => {
			  // if (err) {
				// console.error(err);
				// return;
			  // }
			  // console.log(stdout);
			// });
	});
	
	res.statusCode = 200;
	res.end();
});

app.post('/save', function (req, res) {
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
	
});

app.post('/load', function (req, res) {
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
	
});

app.listen(PORT,'0.0.0.0', function () {
  console.log('Listening on port '+PORT+'!');
});