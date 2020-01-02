package atlasdsl;

public class PointProperty extends ComponentProperty {
	private Point value;
	
	public PointProperty(String name, Point v) {
		this.name = name;
		this.value = v;
	}
	
	public Point getValue() {
		return value;
	}
}
