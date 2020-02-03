package atlassharedclasses;

public class Region {
	private Point point1;
	private Point point2;
	
	public Region(double a, double b, double c, double d) {
		this.point1 = new Point(a,b);
		this.point2 = new Point(c,d);
	}

}
