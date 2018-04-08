/*
Copyright 2018 Ola Benderius

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

var g_pause = false;

$(document).ready(function(){
  
  $('body').on('click', 'button#pause', function() {
    g_pause = !g_pause;
    if (g_pause) {
      $('button#pause').html('Continue');
    } else {
      $('button#pause').html('Pause');
    }
  });
  
  setupUi();
});

function setupUi() {
  var lc = libcluon();

  if ("WebSocket" in window) {
    var ws = new WebSocket("ws://" + window.location.host + "/", "data");
    ws.binaryType = 'arraybuffer';

    ws.onopen = function() {
      onStreamOpen(ws, lc);
    }

    ws.onmessage = function(evt) {
      onMessageReceived(lc, evt.data);
    };

    ws.onclose = function() {
      onStreamClosed();
    };

  } else {
    console.log("Error: websockets not supported by your browser.");
  }

}

function onStreamOpen(ws, lc) {
  function getResourceFrom(url) {
    var xmlHttp = new XMLHttpRequest();
    xmlHttp.open("GET", url, false);
    xmlHttp.send(null);
    return xmlHttp.responseText;
  }

  var odvd = getResourceFrom("opendlv-standard-message-set-v0.9.4.odvd");

  console.log("Connected to stream.");
  console.log("Loaded " + lc.setMessageSpecification(odvd) + " messages from specification.");
  
  function sendMessage(dataType, senderStamp, messageJson) {
    const message = lc.encodeEnvelopeFromJSONWithoutTimeStamps(messageJson, dataType, senderStamp);
    strToAb = str =>
      new Uint8Array(str.split('')
        .map(c => c.charCodeAt(0))).buffer;
    ws.send(strToAb(message), { binary: true });
  }
 
  setupControllerView(sendMessage);
  setupSimulationView();
  setupViewer();
}

function onStreamClosed() {
  console.log("Disconnected from stream.");
}

function onMessageReceived(lc, msg) {

  if (g_pause) {
    return;
  }

  var data_str = lc.decodeEnvelopeToJSON(msg);
  
  if (data_str.length == 2) {
    return;
  }

  d = JSON.parse(data_str);

  // Translate to nice JSON ..
  var payloadFields = new Array();

  const payloadName = Object.keys(d)[5];
  
  for (const fieldName in d[payloadName]) {
    const fieldValue = d[payloadName][fieldName];
    const field = {
      name : fieldName,
      value : fieldValue,
      type : (typeof fieldValue)
    };
    payloadFields.push(field);
  }

  const data = {
    dataType : d.dataType,
    payload : {
      name : payloadName,
      fields : payloadFields
    },
    received : d.received,
    sampleTimeStamp : d.sampleTimeStamp,
    senderStamp : d.senderStamp,
    sent : d.sent
  };
  // .. done.

  addSignalViewerData(data);
  addSimulationViewData(d);
}
