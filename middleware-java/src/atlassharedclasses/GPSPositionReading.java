package atlassharedclasses;

public class GPSPositionReading extends SensorInfo {
	private double x;
	private double y;
	
	// There has to be a default constructor to allow the class to be 
	// serialised.
	public GPSPositionReading() {
		
	}
	
	public GPSPositionReading(double x, double y) {
		this.x = x;
		this.y = y;
	}
	
	public double getX() {
		return x;
	}
	
	public double getY() {
		return y;
	}
}
