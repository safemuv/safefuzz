package middleware.carstranslations;

import java.util.HashMap;

import atlasdsl.Robot;
import middleware.core.ActiveMQProducer;

public interface CARSTranslations {
	public void sendCARSUpdate(String robotName, Object key, Object value);
	public void startRobot(String robotName);
	public void setOutputProducers(HashMap<String,ActiveMQProducer> producers);
	
	//public String translatePropertyToCARSUpdate(String property);
}
