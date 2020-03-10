package atlasdsl;
import atlassharedclasses.Point;

public class PointResultField extends GoalResultField {
	private Point value;
	
	public PointResultField(String name, Point value) {
		this.name = name;
		this.value = value;
	}
		
	public String toString() {
		return name + "=" + value.toString();
	}
	
	public Point getValue() {
		return value;
	}
}
