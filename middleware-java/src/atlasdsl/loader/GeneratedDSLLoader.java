package atlasdsl.loader;

import atlasdsl.*;
import atlasdsl.faults.*;
import atlassharedclasses.*;
import java.util.Optional;
import java.util.List;
import java.util.ArrayList;

public class GeneratedDSLLoader implements DSLLoader {
	public Mission loadMission() throws DSLLoadFailed {
	final double MISSION_END_TIME = 1200.0;
	Mission mission = new Mission(MISSION_END_TIME);
	
	
		Robot rgilda = new Robot("gilda");
		rgilda.setPointComponentProperty("startLocation", new Point(0.0,0.0,0.0));
		rgilda.setDoubleComponentProperty("maxSpeed", 5.0);
		rgilda.setDoubleComponentProperty("startSpeed", 1.5);
		
 
		Sensor srgilda_1 = new Sensor(SensorType.SONAR);
		srgilda_1.setParent(rgilda);
		srgilda_1.setDoubleComponentProperty("swathWidth", 10.0);
		srgilda_1.setDoubleComponentProperty("detectionProb", 0.99);
		rgilda.addSubcomponent(srgilda_1);
			
			
			
			
			
		mission.addRobot(rgilda);
		Robot rhenry = new Robot("henry");
		rhenry.setPointComponentProperty("startLocation", new Point(0.0,0.0,0.0));
		rhenry.setDoubleComponentProperty("maxSpeed", 5.0);
		rhenry.setDoubleComponentProperty("startSpeed", 1.5);
		
 
		Sensor srhenry_1 = new Sensor(SensorType.SONAR);
		srhenry_1.setParent(rhenry);
		srhenry_1.setDoubleComponentProperty("swathWidth", 10.0);
		srhenry_1.setDoubleComponentProperty("detectionProb", 0.99);
		rhenry.addSubcomponent(srhenry_1);
			
			
			
			
			
		mission.addRobot(rhenry);
	
	
	
	
	
	
	

	
	
	
	return mission;
	}
}