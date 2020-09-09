package carsspecific.moos.carsqueue;

import middleware.core.CARSEvent;

public class PShareEvent extends CARSEvent {
	private byte [] contents;
	private int len;
	PShareEvent(byte [] contents, int len) {
		this.contents = contents;
		this.len = len;
	}
	
	public byte [] byteContents() {
		return contents;
	}

	public int byteLen() {
		return len;
	}	
}