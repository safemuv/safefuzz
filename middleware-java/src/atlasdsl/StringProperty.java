package atlasdsl;

public class StringProperty extends ComponentProperty {
	private String value;

	public StringProperty(String name, String value) {
		this.name = name;
		this.value = value;
	}

	public String getValue() {
		return value;
	}
}