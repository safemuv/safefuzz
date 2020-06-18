package middleware.core;

import atlasdsl.Robot;

public interface CARSTranslations {
	public void sendCARSUpdate(String robotName, Object key, Object value);
	public void startRobot(String robotName);
}
