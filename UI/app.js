const path = './demos/';
const fs = require('fs');
var qs = require('querystring');
var express = require('express');
var app = express();

app.use(express.static(__dirname + '/'));

app.get('/', function (req, res) {
	res.sendFile('./index.html', { root: __dirname });
});

app.post('/run', function (req, res) {
	const { headers, method, url } = req;
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
	});
	
	res.statusCode = 200;
	res.end();
});

app.post('/save', function (req, res) {
	const { headers, method, url } = req;
	var body = '';
	const responseBody = { headers, method, url, body };
	
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

app.listen(3000, function () {
  console.log('Example app listening on port 3000!');
});