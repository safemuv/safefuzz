package atlassharedclasses;

public class SonarDetection extends SensorInfo {
	// TODO: test to see if it fixes the serialising of Jackson fields
	public Point detectionLocation;
	public String detectingVehicleName;
	public int objectID;
	
	public SonarDetection() {
		
	}
	
	public Point getLocation() {
		return detectionLocation;
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
	
	public int getObjectID() {
		return objectID;
	}
}
