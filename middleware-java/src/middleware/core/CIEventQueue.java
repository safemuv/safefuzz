package middleware.core;

public class CIEventQueue extends ATLASEventQueue<CIEvent> {
	private static final long serialVersionUID = 1L;

	public CIEventQueue(int capacity) {
		super(capacity, '@');

	}

	public void setup() {

	}
	
	public void handleEvent(CIEvent event) {

	}
}
