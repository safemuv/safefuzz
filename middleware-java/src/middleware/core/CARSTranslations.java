package middleware.core;

import java.util.HashMap;

import atlasdsl.Robot;

public interface CARSTranslations {
	public void sendCARSUpdate(String robotName, Object key, Object value);
	public void startRobot(String robotName);
	public void setOutputProducers(HashMap<String,ActiveMQProducer> producers);
}
