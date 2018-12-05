const path = './demos/';
const pathXML = './demos/xml/';
const pathPy = './demos/src/';
const fs = require('fs');
const { exec } = require('child_process');
const PORT = process.env.port || 8080;
// const PORT = Number(process.argv.slice(2));

var xml2js = require('xml2js')
var qs = require('querystring');
var express = require('express');
var app = express();
var locked = false;
var param = String(process.argv.slice(2));

var isDev = param == 'dev';
var participants = 15;
var tmpcnt = 0;
var blocklyEnabled = false;
var quesEnabled = false;

function loadIndex(req, res) {
	res.sendFile('./index.html', { root: __dirname });
};

function loadQuestionnaire(req, res) {
	res.sendFile('./questionnaire.html', { root: __dirname });
};

function loadHelp(req, res) {
	res.sendFile('./help.html', { root: __dirname });
};

function loadBlockly(req, res) {
	res.sendFile('./blockly.html', { root: __dirname });
};

function loadEditor(req, res) {
	res.sendFile('./editor.html', { root: __dirname });
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
	var demoname = req.params.demoname;

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

		fs.readFile(pathXML + demoname + ".xml", 'utf8', function (err, data) {
			if (err) {
				console.log(err);
				res.status(400).send("Demo " + demoname + " doesn't exist!");
			}
			else {
				res.status(200).send({ "result": data })
			};
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

function loadToolbox(req, res) {

	fs.readFile('./toolbox.xml', function (err, data) {
		var xmlParser = new xml2js.Parser();
		xmlParser.parseString(data, function (err, result) {
			fs.readFile('./blocks.json', function (err, customBlocks) {
				var customBlocks = JSON.parse(customBlocks);

				var customBlocksCat = {
					'$': {
						id: 'catCustomBlocks',
						name: 'Custom Blocks',
						colour: '50'
					},
					'block': [
					]
				};

				for (var i in customBlocks) {
					customBlocksCat.block.push({ '$': { 'type': 'custom' + i.toString() } })
				}
				if (customBlocksCat.block.length > 0) {
					result.xml.category.unshift(customBlocksCat)
				}

				builder = new xml2js.Builder();
				var toolbox = builder.buildObject(result);

				var ret = { "toolbox": toolbox, "blocks": customBlocks };

				res.status(200).send(ret);
			});
		});
	});
}

function loadConfigurator(req, res) {
	res.sendFile('./configurator.html', { root: __dirname });
}

function createBlock(req, res) {
	var body = '';
	var reqpath = req.path;

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
		var code = post['block[code]'];
		var block = post['block[block]'];
		var meta = JSON.parse(post['block[meta]']);
		var id = post['block[id]'];

		var newBlock = { id: id, meta: meta, code: code, block: block }

		fs.readFile('./blocks.json', function (err, exisBlocks) {
			var exisBlocks = JSON.parse(exisBlocks);

			if (reqpath == '/block/create') {
				newBlock.name = "custom" + exisBlocks.length.toString();
				exisBlocks.push(newBlock);
				var action = "created";
			} else {
				var idx = exisBlocks.findIndex(v => v.id == id);
				newBlock.name = exisBlocks[idx].name;
				exisBlocks[idx] = newBlock;
				var action = "updated";
			}


			fs.writeFile("./blocks.json", JSON.stringify(exisBlocks), (err) => {
				if (err) throw err;
				console.log("Block " + id + " " + action);
				res.status(200).send(exisBlocks);
			});

		});
	});
}

function deleteBlock(req, res) {
	var id = req.params.blockId;

	fs.readFile('./blocks.json', function (err, exisBlocks) {
		if (err) res.status(404).send({})
		var exisBlocks = JSON.parse(exisBlocks);
		var idx = exisBlocks.findIndex(v => v.id == id);
		exisBlocks.splice(idx, 1);

		fs.writeFile("./blocks.json", JSON.stringify(exisBlocks), (err) => {
			if (err) throw err;
			console.log("Block " + id + " deleted.");
			res.status(200).send(exisBlocks)
		});

	});
}

function getBlocks(req, res) {
	fs.readFile('./blocks.json', function (err, exisBlocks) {
		if (err) res.status(404).send({})
		var exisBlocks = JSON.parse(exisBlocks);
		res.status(200).send(exisBlocks)
	});
}

function saveSubmission(req, res) {
	var body = '';
	var userId = req.params.userId;

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
		var subm = JSON.parse(post['data']);
		var subtype = subm.type;
		var filepath = './submissions/' + subtype + '.json';

		fs.readFile(filepath, function (err, data) {
			if (err) res.status(500).send({})
			var submissions = JSON.parse(data);
			submissions.push(subm);

			fs.writeFile(filepath, JSON.stringify(submissions), (err) => {
				if (err) res.status(500).send(err);
				console.log("User " + userId + " submitted " + subtype);
				res.status(200).send(submissions)
			});

		});
	});
}

function startStudy(req, res) {
	var cut = parseInt(participants / 2);
	if (tmpcnt < cut) {
		loadBlockly(req, res);
		tmpcnt += 1;
	} else {
		loadEditor(req, res);
	}
}

function checkSubmissions(req, res) {
	var submissions = [];
	var subtypes = req.params.subtype ? [req.params.subtype] : ["code", "blockly", "questionnaire"];

	for (var subtype of subtypes) {
		submissions = submissions.concat(checkSubmissionPerType(subtype))
	}
	submissions = {_total: submissions.length, submissions: submissions};
	
	res.status(200).send(submissions)

}

function checkSubmissionPerType(subtype) {
	var filepath = './submissions/' + subtype + '.json';
	var submissions = JSON.parse(fs.readFileSync(filepath, 'utf8'));
	submissions = submissions.map(v => JSON.stringify({ user: v.user, type: v.type })).map(v => JSON.parse(v));
	return submissions

}

function evaluationMgmt(req, res) {
	if (!blocklyEnabled && !quesEnabled) {
		loadEditor(req,res)
	} else if (blocklyEnabled && !quesEnabled) {
		loadBlockly(req,res)
	} else {
		loadQuestionnaire(req,res)
	}
}

app.use(express.static(__dirname + '/'));
app.use(function(req,res,next){
	console.log(req.path)
	if(req.query.step == 'blockly' && !blocklyEnabled) {
		res.status(401).send("Not now!")
	} else if(req.query.step == 'questions' && !quesEnabled) {
		res.status(401).send("Not now!")
	} else {
		next();
	}
});

app.get('/', loadIndex);
app.get('/editor', loadEditor);
app.get('/blockly', loadBlockly);
app.get('/questionnaire', loadQuestionnaire);
app.get('/help', loadHelp);
app.post('/demo/run', runCode);
app.post('/demo/save', saveWorkspace);
app.get('/demo/load/:demoname', loadDemo);
app.get('/demo/list', showDemolist);
app.post('/demo/delete', deleteDemo);
app.get('/demo/toolbox', loadToolbox);
app.get('/configurator', loadConfigurator);
app.post('/block/create', createBlock);
app.put('/block/update', createBlock);
app.delete('/block/delete/:blockId', deleteBlock);
app.get('/block/list', getBlocks);
app.post('/submit/:userId', saveSubmission);
app.get('/study', startStudy);
app.get('/checksub', checkSubmissions)
app.get('/checksub/:subtype', checkSubmissions)
app.get('/evaluation',evaluationMgmt)

app.get('/start/:step', function(req,res) {
	var step = req.params.step;
	
	if (step=="blockly") {
		blocklyEnabled = true;
	} else if (step=="questions") {
		quesEnabled = true;
	}
	let txt = "Enabled: "+step;
	console.log(txt)
	res.status(200).send(txt)
});

app.get('/end/:step', function(req,res) {
	var step = req.params.step;
	
	if (step=="blockly") {
		blocklyEnabled = false;
	} else if (step=="questions") {
		quesEnabled = false;
	}
	let txt = "Disabled: "+step;
	console.log(txt)
	res.status(200).send(txt)
});

app.listen(PORT, '0.0.0.0', function () {
	console.log('Hobbit blockly is now listening to port ' + PORT + '!');
});