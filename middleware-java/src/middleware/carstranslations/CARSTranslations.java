package middleware.carstranslations;

import java.util.HashMap;
import java.util.List;

import atlassharedclasses.Point;
import middleware.core.*;

public abstract class CARSTranslations {
	public abstract void sendCARSUpdate(String robotName, Object key, Object value);
	public abstract void startRobot(String robotName);
	public abstract void setCoordinates(String robotName, List<Point> coords);
	public abstract void setOutputProducers(HashMap<String,ActiveMQProducer> producers);
	
	public void sendBackEvent(CARSVariableUpdate event, String reflectBackName) {
		String vehicleName = event.getVehicleName();
		String key = reflectBackName;
		String value = event.getValue();
		sendCARSUpdate(vehicleName, key, value);
	}
	
	public abstract void returnHome(String robotName);
}
