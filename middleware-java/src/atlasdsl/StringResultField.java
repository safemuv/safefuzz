package atlasdsl;

public class StringResultField extends GoalResultField {
	private String value;
	
	public StringResultField(String name, String value) {
		this.name = name;
		this.value = value;
	}
		
	public String toString() {
		return name + "=" + value.toString();
	}
	
	public String getValue() {
		return value;
	}
}
