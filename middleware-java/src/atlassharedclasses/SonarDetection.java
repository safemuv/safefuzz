package atlassharedclasses;

import java.util.Optional;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import atlasdsl.*;

public class SonarDetection extends SensorInfo {
	// TODO: test to see if it fixes the serialising of Jackson fields
	public Point detectionLocation;
	public String detectingVehicleName;
	public int objectID;
	
	public SonarDetection() {
		
	}
	
	public SonarDetection(Point detectionLocation, String detectingVehicleName, int objectID) {
		this.detectionLocation = detectionLocation;
		this.detectingVehicleName = detectingVehicleName;
		this.objectID = objectID;
	}
	
	public String toString() {
		return "SENSORDETECTION-SONAR-" + detectionLocation.toStringBareCSV() + "," + detectingVehicleName + "," + Integer.toString(objectID);
	}
	
	public String getRobotName() {
		return detectingVehicleName;
	}
}
