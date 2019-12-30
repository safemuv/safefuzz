package atlasdsl;

public class DoubleProperty extends ComponentProperty {
	private double value;
	
	public DoubleProperty(String name, double v) {
		this.name = name;
		this.value = v;
	}
}
