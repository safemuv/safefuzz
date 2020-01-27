package middleware.core;

import java.util.Optional;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import atlasdsl.*;

public class SensorDetection {
	private Point detectionLocation;
	private Robot detectingVehicle;
	private EnvironmentalObject object;
	
	private static Pattern scanner = Pattern.compile("SENSORDETECTION-([^,]+),([^,]+),([^,]+)"); 
	
	public SensorDetection(Point detectionLocation, Robot detectingVehicle, EnvironmentalObject object) {
		this.detectionLocation = detectionLocation;
		this.detectingVehicle = detectingVehicle;
		this.object = object;
	}
	
	public String toString() {
		return "SENSORDETECTION-" + detectionLocation.toStringBareCSV() + "," + detectingVehicle.getName() + "," + Integer.toString(object.getLabel());
	}
	
	public static Optional<SensorDetection> parse(String text, Mission mission) {
		// Parse these fields
		Matcher res = scanner.matcher(text);
		if (res.find()) {
			Point loc = new Point((Double.parseDouble(res.group(1))),
												(Double.parseDouble(res.group(2))));
			String vehicle = res.group(3);
			Integer label = Integer.parseInt(res.group(4));
			// Need to look up the objects here
			Robot r = mission.getRobot(vehicle);
			Optional<EnvironmentalObject> eo = mission.getEnvironmentalObject(label);
			
			if (eo.isPresent() && r != null)
				return Optional.of(new SensorDetection(loc, r, eo.get()));
				else return Optional.empty();
		} else return Optional.empty();
	}
}
