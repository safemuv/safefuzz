package atlassharedclasses;

import java.util.Optional;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import atlasdsl.*;

public class SonarDetection extends SensorInfo {
	private Point detectionLocation;
	private String detectingVehicleName;
	private int objectID;
	
	private static Pattern scanner = Pattern.compile("SENSORDETECTION-SONAR-([^,]+),([^,]+),([^,]+),([^,]+)"); 
	
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
	
	public static Optional<SonarDetection> parse(String text, Mission mission) {
		// Parse these fields
		Matcher res = scanner.matcher(text);
		if (res.find()) {
			Point loc = new Point((Double.parseDouble(res.group(1))),
												(Double.parseDouble(res.group(2))));
			String vehicle = res.group(3);
			Integer objectID = Integer.parseInt(res.group(4));
			return Optional.of(new SonarDetection(loc, vehicle, objectID));
		} else return Optional.empty();
	}
}
