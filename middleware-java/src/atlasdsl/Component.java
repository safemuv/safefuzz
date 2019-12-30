package atlasdsl;

import java.util.ArrayList;

public class Component {
	protected String name;
	// TODO: replace with Map
	private ArrayList<ComponentProperty> properties = new ArrayList<ComponentProperty>();
	
	public void setIntComponentProperty(String name, int value) {
		properties.add(new IntProperty(name, value));
	}
	
	public void setDoubleComponentProperty(String name, double value) {
		properties.add(new DoubleProperty(name, value));
	}
}
