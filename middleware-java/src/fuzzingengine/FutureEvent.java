package fuzzingengine;

import java.util.Optional;

public class FutureEvent<E> implements Comparable<FutureEvent<E>> {
	private E e;
	private double pendingTime;
	private Optional<String> reflectBackName;

	public FutureEvent(E e, double pendingTime, Optional<String> reflectBackName) {
		this.e = e;
		this.pendingTime = pendingTime;
		this.reflectBackName = reflectBackName;
	}

	public double getPendingTime() {
		return pendingTime;
	}

	public E getEvent() {
		return e;
	}
	
	public Optional<String> getReflectBackName() {
		return reflectBackName;
	}

	public int compareTo(FutureEvent<E> other) {
		return Double.compare(this.pendingTime, other.pendingTime);
	}

}