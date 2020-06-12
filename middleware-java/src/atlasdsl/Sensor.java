package atlasdsl;

public class Sensor extends Subcomponent {
	
	private SensorType sensorType;
	
	public Sensor(SensorType sensor_type) {
		this.sensorType = sensor_type;
	}
	
	public SensorType getType() {
		return sensorType;
	}
	
	public static String sensorTypeToString(SensorType st) {
		return st.name();
	}
}