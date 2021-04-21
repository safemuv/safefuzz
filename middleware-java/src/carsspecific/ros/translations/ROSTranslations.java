package carsspecific.ros.translations;

import java.util.HashMap;
import java.util.List;
import atlassharedclasses.Point;
import middleware.carstranslations.CARSTranslations;
import middleware.core.ActiveMQProducer;

public class ROSTranslations extends CARSTranslations {
	HashMap<String,ActiveMQProducer> producers;
	
	public ROSTranslations() {

	}
	
	public void setOutputProducers(HashMap<String,ActiveMQProducer> producers) {
		this.producers = producers;
	}
	
	public synchronized void sendCARSUpdate(String robotName, Object key, Object value) {
		System.out.println("ROSTranslations: sendCARSUpdate unimplemented");
	}
	
	public synchronized void vehicleStatusChange(String robotName, boolean newStatus) { 
		System.out.println("ROSTranslations: startRobot unimplemented");
	}

	public void setCoordinates(String robotName, List<Point> coords) {
		System.out.println("ROSTranslations: setCoordinates unimplemented");	
	}

	public void returnHome(String robotName) {
		System.out.println("ROSTranslations: setCoordinates unimplemented");			
	}

	public void setCoordinates(String robotName, List<Point> coords, int repeatCount) {
		
	}

	public void startVehicle(String robotName) {
		
	}

	public void stopVehicle(String robotName) {
		
	}
}
