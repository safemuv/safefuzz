package atlasdsl;

public class Point {
	private double x;
	private double y;
	private double z = 0;
	
	public double getX() {
		return x;
	}
	
	public double getY() {
		return y;
	}
	
	public double getZ() {
		return z;
	}
	
	// Empty constructor since Point may be used in SonarDetection
	// and things serialised by Jackson need it
	public Point() {
		
	}
	
	public Point(double x, double y) {
		this.x = x;
		this.y = y;
	}
	
	public Point(double x, double y, double z) {
		this.x = x;
		this.y = y;
		this.z = z;
	}
	
	public String toString() {
		return "x=" + x + ",y=" + y;
	}
	
	public String toStringBareCSV() {
		return x + "," + y;
	}
}
