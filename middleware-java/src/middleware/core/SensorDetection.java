package middleware.core;

import atlasdsl.*;

public class SensorDetection {
	private Point detectionLocation;
	private Robot detectingVehicle;
	private EnvironmentalObject object;
	
	public SensorDetection(Point detectionLocation, Robot detectingVehicle, EnvironmentalObject object) {
		this.detectionLocation = detectionLocation;
		this.detectingVehicle = detectingVehicle;
		this.object = object;
	}
	
	public String toString() {
		return detectionLocation.toString() + "-" + detectingVehicle.getName() + "-" + Integer.toString(object.getLabel());
	}
}
