package middleware.carstranslations;

import java.util.HashMap;

import middleware.core.*;

public abstract class CARSTranslations {
	public abstract void sendCARSUpdate(String robotName, Object key, Object value);
	public abstract void startRobot(String robotName);
	public abstract void setOutputProducers(HashMap<String,ActiveMQProducer> producers);
	
	public void sendBackEvent(CARSVariableUpdate event) {
		String vehicleName = event.getVehicleName();
		String key = event.getKey();
		String value = event.getValue();
		sendCARSUpdate(vehicleName, key, value);
	}
}
