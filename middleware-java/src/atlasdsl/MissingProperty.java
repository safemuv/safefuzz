package atlasdsl;

public class MissingProperty extends Exception {
	private String propertyName;
	private Component component;
	
	public MissingProperty(Component thing, String propertyName) {
		this.component = thing;
		this.propertyName = propertyName;
	}
	
	public String getPropertyName() {
		return propertyName;
	}
	
	public Component getComponent() {
		return component;
	}
	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;

}
