package atlassharedclasses;

public class GPSPositionReading extends SensorInfo {
	private double x;
	private double y;
	private String robotName;
	
	// There has to be a default constructor to allow the class to be 
	// serialised.
	public GPSPositionReading() {
		
	}
	
	public GPSPositionReading(double x, double y, String robotName) {
		this.x = x;
		this.y = y;
		this.robotName = robotName;
	}
	
	public double getX() {
		return x;
	}
	
	public double getY() {
		return y;
	}
	
	public String getRobotName() {
		return robotName;
	}
}
