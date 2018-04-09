var coneIdArray = [];
var coneMap = new Map();

function addSimulationConeViewData(d, data){
// ObjectDirection 1133
// ObjectDistance 1134
  if (data.dataType == 1133) {
    const id = data["opendlv_logic_perception_ObjectDirection"]["objectId"];
    const azimuthAngle = data["opendlv_logic_perception_ObjectDirection"]["azimuthAngle"];
    const zenithAngle = data["opendlv_logic_perception_ObjectDirection"]["zenithAngle"];
    if(!coneMap.has(id)){
    	coneMap.set(id, new Array());
    	coneIdArray.push(id);
    }
    storeConeData(id, d, data);
    // store data depends on time
    /*
    coneArray[id][0] = azimuthAngle;
    coneArray[id][1] = zenithAngle;
    */
  }
  if (data.dataType == 1134) {
    
    const id = data["opendlv_logic_perception_ObjectDistance"]["objectId"];
    const distance = data["opendlv_logic_perception_ObjectDistance"]["distance"];
    
    //coneArray[id][2] = distance;
    
    if(!coneMap.has(id)){
      coneMap.set(id, new Array());
      coneIdArray.push(id);
    }
    storeConeData(id, d, data);
  }

}

function storeConeData(id, d, data){
	const newConeTime = data.sent.seconds;
	for(var i = 0; i < coneIdArray.length; i++){
		const preConeId = coneIdArray[i];
		const preConeData = coneMap.get(preConeId);
		const preConeDataTime = preConeData.sent.seconds;
		const age = newConeTime - preConeDataTime;
		
		document.write(age); 
	}
	coneMap.get(id).push(data);
}