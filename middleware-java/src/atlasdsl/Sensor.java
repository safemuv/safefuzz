package atlasdsl;

public class Sensor extends Subcomponent {
	
	private SensorType sensor_type;
	
	public Sensor(SensorType sensor_type) {
		this.sensor_type = sensor_type;
	}
}