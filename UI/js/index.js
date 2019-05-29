function listDemos() {
    $("#cont").empty();
    $.get("/demo/list",
        {
        }, function (data, status) {
            if (status == "success") {
                for (var demofile of data.result) {
                    var title = demofile.slice(0, -4);
                    var cardClass = 'class="col s12 m6 l4 xl3"';

                    var demos = [
                        '<div style="height: 140px;margin-bottom: 24px"' + cardClass + '>',
                        '<div class="card">',
                        '<div class="valign-wrapper demo">',
                        '<div class="card-content">',
                        '<div><h5>' + title + '</h5></div>',
                        '<div class="row center-align">',
                        '<a><div class="col s4 m4 truncate" onclick={deleteDemo(\'' + title + '\')}>delete</div></a>',
                        '<a><div class="col s4 m4" onclick={runDemo(\'' + title + '\')}>run</div></a>',
                        '<a><div class="col s4 m4" onclick={showDemo(\'' + demofile + '\')}>edit</div></a>',
                        '</div>', '</div>', '</div>', '</div>', '</div>'
                    ].join("\n");

                    $("#cont").append(demos);
                }
                var createDemo = [
                    '<div ' + cardClass + '>',
                    '<div class="card">',
                    '<div class="valign-wrapper call-blockly" onclick={loadBlockly()}>',
                    '<div style="margin: 0 auto;">+</div>',
                    '</div>', '</div>', '</div>'
                ].join("\n");

                var configurator = [
                    '<div ' + cardClass + '>',
                    '<div class="card">',
                    '<div class="valign-wrapper call-blockly" onclick={loadConfigurator()}>',
                    '<div style="margin: 0 auto;"><img src="./img/config-icon.png"></div>',
                    '</div>', '</div>', '</div>'
                ].join("\n");

                $("#cont").append(createDemo);
                $("#cont").append(configurator);
            } else {
                alert("Something went wrong!");
            }
        });
}

function showDemo(demofile) {
    $.get("/demo/load/"+demofile.slice(0,-4),
        {
        }, function (data, status) {
            if (status == "success") {
                xml_demo = data.result;
                localStorage.setItem("blocks_cache", xml_demo);
                loadBlockly(true);
            } else {
                alert("Something went wrong!");
            }
        });
}

function deleteDemo(demoname) {
    if (confirm("Do you really want to delete this demo?") == true)
        $.post("/demo/delete",
            {
                demoname: demoname
            }, function (data, status) {
                if (status == "success") {
                    location.reload()
                    console.log("Demo \"" + demoname + "\" sucessfully deleted.");
                } else {
                    alert("Something went wrong!");
                }
            });
}

function runDemo(demofile) {
    if (confirm("Do you really want to run the demo?") == true) {
        $.post("/demo/run",
            {
                filename: "run.py",
                sourcepath: "./demos/src/",
                sourcefile: demofile + ".py",
                main: true
            }, function (data, status) {
                if (status == "success") {
                    alert(data.result);
                } else {
                    alert("Something went wrong!");
                }
            });
    }
}

function loadBlockly(keepStorage) {
    var url = window.location.href + 'blockly';
    if (!keepStorage) {
        localStorage.removeItem("blocks_cache");
    }
    window.location.href = url;
}

function loadConfigurator() {
    var url = window.location.href + 'configurator';
    window.location.href = url;
}
