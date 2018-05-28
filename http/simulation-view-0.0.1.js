/*
Copyright 2018 Ola Benderius

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

var g_centerNext = true;
var g_scale = 100;
var g_scrollX = 0;
var g_scrollY = 0;
var g_walls = [];


var g_scaleSim = 100;
var g_scrollXSim = 0;
var g_scrollYSim = 0;


function setupSimulationView() {
  function getResourceFrom(url) {
    var xmlHttp = new XMLHttpRequest();
    xmlHttp.open("GET", url, false);
    xmlHttp.send(null);
    return xmlHttp.responseText;
  }

var map = getResourceFrom("trackTest2R.csv");
map.trim().split("\n").forEach(function(wall) {
  const wallArray = wall.trim().split(",");
  if (wallArray.length == 3) {
    g_walls.push(wallArray);
  }
});

  /*
  var map = getResourceFrom("simulation-map-closed-lap.txt");
  map.trim().split(";").forEach(function(wall) {
    const wallArray = wall.trim().split(",");
    if (wallArray.length == 4) {
      g_walls.push(wallArray);
    }
  });
  */

  var clicked = false;
  var clickX;
  var clickY;
  $('#simulation-canvas').on({
    'mousemove': function(e) {
      if (clicked) {
        const deltaX = clickX - e.pageX;
        const deltaY = clickY - e.pageY;

        if (e.ctrlKey) {
          g_scale += deltaY;
          if (g_scale < 1) {
            g_scale = 1;
          }
          if (g_scale > 500) {
            g_scale = 500;
          }

          g_scaleSim += deltaY;
          if (g_scaleSim < 1) {
            g_scaleSim = 1;
          }
          if (g_scaleSim > 500) {
            g_scaleSim = 500;
          }
        } else {
          g_scrollX += deltaX;
          g_scrollY += deltaY;
          g_scrollXSim += deltaX;
          g_scrollYSim += deltaY;
        }


        clickX = e.pageX;
        clickY = e.pageY;
      }
    },
    'mousedown': function(e) {
      $('html').css('cursor', 'all-scroll');
      clicked = true;
      clickX = e.pageX;
      clickY = e.pageY;
    },
    'mouseup': function() {
      clicked = false;
      $('html').css('cursor', 'auto');
    },
    'dblclick': function(e) {
      g_centerNext = true;
    }
  });
}
var headingRequest =0;
var Vx=0;
var Vy=0;
var Ax=0;
var AxReq=0;
var groundSpeed=0;
var distanceToAimPoint = 0;
var Ay =0;
var yawRateDot =0;
var yawRate =0;
function addSimulationViewData(data) {
  if (data.dataType == 1172){
    headingRequest = data["opendlv_logic_action_AimPoint"]["azimuthAngle"];
    distanceToAimPoint = data["opendlv_logic_action_AimPoint"]["distance"];
  }
  if (data.dataType == 1046){
    groundSpeed = data["opendlv_proxy_GroundSpeedReading"]["groundSpeed"];
  }
  if (data.dataType == 1002){
    Vx = data["opendlv_sim_KinematicState"]["vx"];
    Vy = data["opendlv_sim_KinematicState"]["vy"];
  }
  if (data.dataType == 1092){
    AxReq = data["opendlv_proxy_GroundAccelerationRequest"]["groundAcceleration"];
  }
  if (data.dataType == 1093){
    AxReq = -data["opendlv_proxy_GroundDecelerationRequest"]["groundDeceleration"];
  }
  if (data.dataType == 1017){
    Ax= data["opendlv_logic_sensation_Equilibrioception"]["vz"];
    Ay = data["opendlv_logic_sensation_Equilibrioception"]["rollRate"];
    yawRateDot = data["opendlv_logic_sensation_Equilibrioception"]["pitchRate"];
    yawRate = data["opendlv_logic_sensation_Equilibrioception"]["yawRate"];
  }
  if (data.dataType == 1001) {
    const x = data["opendlv_sim_Frame"]["x"];
    const y = data["opendlv_sim_Frame"]["y"];
    const yaw = data["opendlv_sim_Frame"]["yaw"];

    const width = 1.3;
    const length = 2;

    var canvas = document.getElementById("simulation-canvas");
    var context = canvas.getContext("2d");

    if (g_centerNext) {
      g_scrollX = -canvas.width / 2;
      g_scrollY = -canvas.height / 2;
      g_scrollXSim = -canvas.width / 2;
      g_scrollYSim = -canvas.height / 2;
      g_centerNext = false;
    }

    const sx = g_scale * x - g_scrollX;
    const sy = -g_scale * y - g_scrollY;
    const swidth = g_scale * width;
    const slength = g_scale * length;
    const syaw = -yaw;

    const hslength = slength / 2;
    const hswidth = swidth / 2;
    const fslength = slength / 4;

    context.clearRect(0, 0, canvas.width, canvas.height);

    context.save();
    context.beginPath();
    context.lineWidth = 2;
    context.translate(sx, sy);
    context.rotate(syaw);
    context.rect(-hslength, -hswidth, slength, swidth);
    context.moveTo(fslength, hswidth);
    context.lineTo(hslength, 0);
    context.moveTo(fslength, -hswidth);
    context.lineTo(hslength, 0);

    // DRAW HEADINGREQUEST
    const H = g_scale*distanceToAimPoint;
    context.moveTo(0.765*g_scale,0);
    context.lineTo(H*Math.cos(-headingRequest),H*Math.sin(-headingRequest));
    //####################
    context.strokeStyle = "black";

    context.stroke();
    context.restore();

    context.save();
    // Outputs
    context.fillStyle="black";
    context.font = "20px Courier New";
    /*context.fillText("X: "+x.toFixed(2)+" | Y: "+y.toFixed(2)+" | Yaw: "+yaw.toFixed(2)+" | Speed: "+groundSpeed.toFixed(2)+" | Ax: "+Ax.toFixed(2), 50, canvas.height-60);
    context.fillText("Vx: "+Vx.toFixed(2)+" | Vy: "+Vy.toFixed(2)+" | Aim: "+headingRequest.toFixed(2)+" | AxReq: "+AxReq.toFixed(2), 50, canvas.height-30); //*/
    context.fillText("Vx: "+Vx.toFixed(2)+" | Vy: "+Vy.toFixed(2) +" | Ax: "+Ax.toFixed(2) +" | Ay: "+Ay.toFixed(2), 50, canvas.height-60); //
    context.fillText("AxReq: "+AxReq.toFixed(2)+" | Aim: "+headingRequest.toFixed(2), 50, canvas.height-30);
    //#####################
    context.restore();

    /*
    for (const wallKey in g_walls) {
      const sx1 = g_scale * g_walls[wallKey][0] - g_scrollX;
      const sy1 = -g_scale * g_walls[wallKey][1] - g_scrollY;
      const sx2 = g_scale * g_walls[wallKey][2] - g_scrollX;
      const sy2 = -g_scale * g_walls[wallKey][3] - g_scrollY;

      context.save();
      context.beginPath();
      context.lineWidth = 5;
      context.moveTo(sx1, sy1);
      context.lineTo(sx2, sy2);
      context.stroke();
      context.restore();
    }
    */
    for (const wallKey in g_walls) {
      const sx1 = g_scaleSim * g_walls[wallKey][0] - g_scrollXSim;
      const sy1 = -g_scaleSim * g_walls[wallKey][1] - g_scrollYSim;
      const type = g_walls[wallKey][2];

      //context.lineWidth = "0.1";
      var coneSize;
      context.beginPath();
      if(type == 1){
        context.fillStyle = "#003cb3";
        coneSize = 6;//0.22*g_scale;
      }
      if(type == 2){
        context.fillStyle = "#ffbf00";
        coneSize = 6;//0.22*g_scale;
      }
      if(type == 3){
        context.fillStyle = "#ff9000";
        coneSize = 6;//0.22*g_scale;
      }
      if(type == 4){
        context.fillStyle = "#ff9000";
        coneSize = 10;//0.3*g_scale;
      }
      context.save();
      context.fillRect(sx1, sy1, coneSize, coneSize);
      context.stroke();
      context.restore();
    }
  }
}
