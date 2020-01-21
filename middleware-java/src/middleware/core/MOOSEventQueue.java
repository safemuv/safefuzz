package middleware.core;

public class MOOSEventQueue extends CARSEventQueue<MOOSEvent> {
	private static final long serialVersionUID = 1L;

	public MOOSEventQueue(int capacity) {
		super(capacity);
	}	
}
