package middleware.core;

import atlassharedclasses.SonarDetection;

public interface SensorDetectionLambda {
	// TODO: change this for general sensor detection, not just Sonar
	public void op(SonarDetection d);
}
