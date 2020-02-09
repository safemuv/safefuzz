package middleware.core;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ArrayBlockingQueue;

import atlassharedclasses.ATLASObjectMapper;

public abstract class ATLASEventQueue<E> extends ArrayBlockingQueue<E> implements Runnable {
	
	public interface VoidLambda {
		public void op();
	}
	
	private static final long serialVersionUID = 1L;
	protected ATLASObjectMapper atlasOMapper;
	private boolean continueLoop = true;
	private char progressChar;
	private List<VoidLambda> afterHooks = new ArrayList<VoidLambda>();

	public ATLASEventQueue(int capacity, char progressChar) {
		super(capacity);
		progressChar = '.';
		this.progressChar = progressChar;
	}
	
	public void registerAfterHook(VoidLambda v) {
		afterHooks.add(v);
	}
	
	public abstract void setup();
	
	public void run() {
		System.out.println("run() in ATLASEventQueue - " + this.hashCode());
		while (continueLoop) {
			try {
				E e = take();
				for (VoidLambda v : afterHooks) {
					v.op();
				}
				System.out.print(progressChar);
				if (e != null) {
					handleEvent(e);
					// TODO: put logger calls here for the new event
				}
			} catch (InterruptedException e) {
				e.printStackTrace();
			}	
		}
	}
	
	public abstract void handleEvent(E event);
}
