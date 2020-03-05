package atlasdsl;

public class DoubleResultField extends GoalResultField {
	private double value;

	public DoubleResultField(String name, Double value) {
		this.name = name;
		this.value = value;
	}
	
	public String toString() {
		return this.name + "=" + Double.toString(value);
	}

}
