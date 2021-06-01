package fuzzexperiment.runner.metrics;

import middleware.core.CARSEvent;

public abstract class OnlineMetric extends Metric {
	public abstract void notifyEvent(CARSEvent e);
	public abstract void logResultAtEnd();
}
