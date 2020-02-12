package middleware.core;

// TODO: This is not used yet - the faults are directly injected into the
// middleware process, rather than over ActiveMQ
public class FaultgenEventQueue extends ATLASEventQueue<FaultEvent> {

	private static final long serialVersionUID = 1L;

	public FaultgenEventQueue(int capacity) {
		super(capacity, '!');
	}

	public void setup() {

	}

	public void handleEvent(FaultEvent event) {
		
	}
}
