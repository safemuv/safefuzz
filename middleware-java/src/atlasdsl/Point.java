package atlasdsl;

public class Point {
	private double x;
	private double y;
	private double z = 0;
	
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
}
