package atlassharedclasses;

public class Region {
	private Point point1;
	private Point point2;
	
	public Region(double a, double b, double c, double d) {
		this.point1 = new Point(a,b);
		this.point2 = new Point(c,d);
	}

	public double width() {
		return Math.abs(point1.getX() - point2.getX());
	}

	public double height() {
		return Math.abs(point1.getX() - point2.getX());		
	}
	
	public double left() {
		return Math.min(point1.getX(), point2.getX());
	}
	
	public double bottom() {
		return Math.max(point1.getY(), point2.getY());
	}
	
	public double right() {
		return Math.max(point1.getX(), point2.getX());
	}
	
	public double top() {
		return Math.min(point1.getY(), point2.getY());
	}
}
