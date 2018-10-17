function addListener() {
    var coll = document.getElementsByClassName("blockly-collapsible");
    var i;

    for (i = 0; i < coll.length; i++) {
        coll[i].addEventListener("click", toggle);
    }
}

function removeListener() {
    var coll = document.getElementsByClassName("blockly-collapsible");
    var i;

    for (i = 0; i < coll.length; i++) {
        coll[i].removeEventListener("click", toggle);
    }
}

function toggle () {
    this.classList.toggle("col-active");
    var content = this.nextElementSibling;
    if (content.style.display === "block") {
        content.style.display = "none";
    } else {
        content.style.display = "block";
    }
}

function updateListener() {
    removeListener();
    addListener();
}

updateListener()