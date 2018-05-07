var coneIdArray = [];
var g_coneMap = new Map();
var g_typeMap = new Map();
var g_feildArray = new Array();
var g_age_microseconds = 10000;
var g_coneLocal = new Array();
var g_coneNum = 0;
var preData = null;
var id;
// x,y
var g_conePosition = new Array([],[]);

const DEG2RAD = Math.PI/180; // PI/180.0
const g_scale_cones = 80;
const g_x_move = 300;
const g_y_move = 800;

function addSimulationConeViewData(d, data){
// ObjectDirection 1133
// ObjectDistance 1134
if (data.dataType == 1133 || data.dataType == 1134){
  if(data.dataType == 1133){
    id = d["opendlv_logic_perception_ObjectDirection"]["objectId"];
  }
  if(data.dataType == 1134){ 
    id = d["opendlv_logic_perception_ObjectDistance"]["objectId"]; 
  }
  /*
  console.log("receive")
  console.log(id);
  console.log(data.dataType);
  */
 
    if(g_coneMap.size == 0){ 
      g_coneMap.set(id, new Map());     
      storeMessage(d, data, id);
    }else{
      if(!g_coneMap.has(id)){
        g_coneMap.set(id, new Map());
      } 
      var age = (data.sampleTimeStamp.seconds - preData.sampleTimeStamp.seconds)*1000000 + (data.sampleTimeStamp.microseconds - preData.sampleTimeStamp.microseconds);
      console.log(data.sampleTimeStamp);
      console.log(age);
      if(age < g_age_microseconds){
        storeMessage(d, data, id);
      }else{
        drawMap();
        g_coneMap = new Map();
        g_coneMap.set(id, new Map());
        g_coneNum = 0;
        g_coneLocal = new Array();
        storeMessage(d, data, id);
      }
    }
    preData = data;
  }
}

function storeMessage(d, data, id){
  if(data.dataType == 1133){
    g_coneMap.get(id).set(1133, new Array());
    const azimuthAngle = d["opendlv_logic_perception_ObjectDirection"]["azimuthAngle"];
    const zenithAngle = d["opendlv_logic_perception_ObjectDirection"]["zenithAngle"];
    var temp = g_coneMap.get(id);
    temp.get(1133).push(azimuthAngle);
    temp.get(1133).push(zenithAngle);
  }
  if(data.dataType == 1134){
    g_coneMap.get(id).set(1134, new Array());
    const distance = d["opendlv_logic_perception_ObjectDistance"]["distance"];
    var temp = g_coneMap.get(id);
    temp.get(1134).push(distance); 
  }
  // reveice both direction and distance
  if(g_coneMap.get(id).size == 2){
    console.log("I am storing");
    // zenithAngle
    g_coneLocal.push([]);
    g_coneLocal[g_coneNum][0] = g_coneMap.get(id).get(1133).pop();
    // azimuthAngle
    g_coneLocal[g_coneNum][1] = g_coneMap.get(id).get(1133).pop();
    // distance
    g_coneLocal[g_coneNum][2] = g_coneMap.get(id).get(1134).pop();

    g_coneNum = g_coneNum + 1;
    
    var showArray = g_coneLocal.slice();
    /*
    console.log(showArray);
    console.log("array length");
    console.log(g_coneLocal.length);
    console.log(g_coneNum);  
    */
    
  }
}

function drawMap(){
  g_conePosition = new Array([],[]);
  extractPosition();

  var canvas = document.getElementById("simulation-cone-canvas");
  var context = canvas.getContext("2d");

  context.clearRect(0, 0, canvas.width, canvas.height);
  
  for(var i = 0; i < g_coneNum; i++ ){
    context.save();
    context.beginPath();
    context.arc(g_x_move + g_scale_cones*g_conePosition[0][i], g_y_move - g_scale_cones*g_conePosition[1][i], 8, 0, 2 * Math.PI);
    context.fillStyle = "blue";
    context.fill();
    context.stroke();
    context.restore();
  }
}

function extractPosition(){
  for(var i = 0; i < g_coneLocal.length; i++){
    var tempP = Spherical2Cartesian(g_coneLocal[i][1], g_coneLocal[i][0], g_coneLocal[i][2]);
    g_conePosition[0][i] = tempP[0];
    g_conePosition[1][i] = tempP[1];
  }
  var showConePosition = g_conePosition.slice();
  /*
  console.log("here are cone positions");
  console.log(showConePosition);
  */
}

function Spherical2Cartesian(azimuth, zenimuth, distance){
  xData = distance * Math.cos(zenimuth * DEG2RAD)*Math.sin(azimuth * DEG2RAD);
  yData = distance * Math.cos(zenimuth * DEG2RAD)*Math.cos(azimuth * DEG2RAD);
  /*
  console.log(azimuth);
  console.log(zenimuth);
  console.log(distance);
  console.log(DEG2RAD);

  console.log("xData");
  console.log(xData);
  console.log("yData");
  console.log(yData);
  */
  var position = new Array();
  position[0] = xData;
  position[1] = yData;
  return position;
}


