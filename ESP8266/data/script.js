var gateway = `ws://${window.location.hostname}/ws`;
var websocket;
var lastSendTime = 0;
const throttleDelay = 50;

function onload(event) {
    initWebSocket();
}

function getValues(){
    websocket.send("getValues");
}

function initWebSocket() {
    websocket = new WebSocket(gateway);
    websocket.onopen = onOpen;
    websocket.onclose = onClose;
    websocket.onmessage = onMessage;
}

function onOpen(event) {
    getValues();
}

function onClose(event) {
    setTimeout(initWebSocket, 2000);
}

function updateSliderPWM(element) {
    const now = Date.now();
    if (now - lastSendTime < throttleDelay) return;
    lastSendTime = now;
    
    var sliderNumber = element.id.charAt(element.id.length-1);
    var sliderValue = element.value;
    document.getElementById("sliderValue"+sliderNumber).innerHTML = sliderValue;
    websocket.send(sliderNumber+"s"+sliderValue);
}

function onMessage(event) {
    var myObj = JSON.parse(event.data);
    var keys = Object.keys(myObj);
    for (var i = 0; i < keys.length; i++){
        var key = keys[i];
        document.getElementById(key).innerHTML = myObj[key];
        document.getElementById("slider"+(i+1)).value = myObj[key];
    }
}

window.addEventListener('load', onload);