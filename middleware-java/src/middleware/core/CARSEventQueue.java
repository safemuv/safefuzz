package middleware.core;

import java.util.concurrent.ArrayBlockingQueue;

public abstract class CARSEventQueue<E> extends ArrayBlockingQueue<E> {
	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;

	public CARSEventQueue(int capacity) {
		super(capacity);
	}
}
