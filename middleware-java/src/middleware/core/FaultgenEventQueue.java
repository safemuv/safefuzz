package middleware.core;

public class FaultgenEventQueue extends ATLASEventQueue<FaultEvent> {

	private static final long serialVersionUID = 1L;

	public FaultgenEventQueue(int capacity) {
		super(capacity, '!');
		// TODO Auto-generated constructor stub
	}

	public void setup() {
		// TODO Auto-generated method stub

	}

	public void handleEvent(FaultEvent event) {
		
	}
}
