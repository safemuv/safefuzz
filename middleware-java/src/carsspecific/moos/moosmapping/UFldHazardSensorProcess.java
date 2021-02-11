package carsspecific.moos.moosmapping;

import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import atlasdsl.*;

public class UFldHazardSensorProcess extends MOOSProcess {
	private List<EnvironmentalObject> objects = new ArrayList<EnvironmentalObject>(); 
	
	
	public UFldHazardSensorProcess(MOOSCommunity parent) {
		super("uFldHazardSensor", parent);
		setProperty("hazard_file", "hazards.txt");
		
		// TODO: factor these out into the DSL
		// These represent a curve which is used to configure detection probabilities
		setProperty("sensor_config", "width=25, exp=4, pclass=0.80");
		setProperty("sensor_config", "width=50, exp=2, pclass=0.60");

		// Temporary fix to add additional properties for the sensor widths
		for (int w = 2; w < 25; w++) {
			setProperty("sensor_config", "width=" + w + ", exp=4, pclass=0.93");
		}
		
		setProperty("min_reset_interval", 0.0001);
		setProperty("max_turn_rate", 10.0);
	}
	
	public void addObject(EnvironmentalObject eo) {
		objects.add(eo);
	}
	
	public void generateCustomCode(MOOSFiles mf) throws IOException {
		FileWriter fw = mf.getOpenFile("hazards.txt");
		for (EnvironmentalObject eo : objects) {
			fw.write(eo.toString() + "\n");
		}
	}
}