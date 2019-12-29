package atlasdsl;

import java.util.ArrayList;

public class Component {
	protected String name;
	private ArrayList<ComponentProperty> properties;
	
	public void setIntComponentProperty(String name, int value) {
		properties.add(new IntProperty(name, value));
	}
}
