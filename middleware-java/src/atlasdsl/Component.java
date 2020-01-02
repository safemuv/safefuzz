package atlasdsl;

import java.util.HashMap;
import java.util.Map;

public class Component {
	protected String name;
	
	private Map<String,IntProperty> intProperties = new HashMap<String,IntProperty>();
	private Map<String,StringProperty> stringProperties = new HashMap<String,StringProperty>();
	private Map<String,DoubleProperty> doubleProperties = new HashMap<String,DoubleProperty>();
	private Map<String,PointProperty> pointProperties = new HashMap<String,PointProperty>();
	
	public void setIntComponentProperty(String name, int value) {
		intProperties.put(name, new IntProperty(name, value));
	}
	
	public void setDoubleComponentProperty(String name, double value) {
		doubleProperties.put(name, new DoubleProperty(name, value));
	}
	
	public void setStringComponentProperty(String name, String value) {
		stringProperties.put(name, new StringProperty(name,value));
	}
	
	public void setPointComponentProperty(String name, Point value) {
		pointProperties.put(name, new PointProperty(name,value));
	}
	
	public int getIntComponentProperty(String name) throws MissingProperty {
		IntProperty p = intProperties.get(name);
		if (p == null) {
			throw new MissingProperty(this,name);
		} else return p.getValue();
	}
	
	public Point getPointComponentProperty(String name) throws MissingProperty {
		PointProperty p = pointProperties.get(name);
		if (p == null) {
			throw new MissingProperty(this,name);
		} else return p.getValue();
	}
	
	public String getStringComponentProperty(String name) throws MissingProperty {
		StringProperty p = stringProperties.get(name);
		if (p == null) {
			throw new MissingProperty(this,name);
		} else return p.getValue();
	}
	
	public double getDoubleComponentProperty(String name) throws MissingProperty {
		DoubleProperty p = doubleProperties.get(name);
		if (p == null) {
			throw new MissingProperty(this,name);
		} else return p.getValue();
	}
}
