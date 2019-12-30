package atlasdsl;

public class Sensor extends Subcomponent {
	private int sensor_type;
	// TODO: better solution here
	public static int SENSE_SONAR = 1; 
	
	public Sensor(int sensor_type) {
		this.sensor_type = sensor_type;
	}
}