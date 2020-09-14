package carsspecific.moos.carsqueue;

import java.io.UnsupportedEncodingException;
import java.util.Arrays;

import middleware.core.CARSEvent;
import middleware.logging.DebugLog;

public class PShareEvent extends CARSEvent {
	
	//private class StringPtr {
//		private int value;
//		
///		public StringPtr(int startVal) {
//			this.value = startVal;
//		}
//		
//		private int get() {
//			return this.value;
//		}
//		
//		public void advance(int by) {
//			this.value += by;
//		}
//	}
	
	// The raw contents of the byte array
	private byte [] contents;
	private int len;
	
	String componentName;
	String communityName;
	String key;
	String value;
	// TODO: what about the double value?
	
	private void decodeMessage() throws UnsupportedEncodingException {
		final int DECODE_POS_START = 14;
		byte[] srcArray = Arrays.copyOfRange(contents, DECODE_POS_START, len);
		String strs = new String(srcArray, "US-ASCII");
		String [] res = strs.split("\0");
		System.out.println("res length = " + res.length);
		
		if (res.length < 25) {
			componentName = res[0];
			communityName = res[7];
			key = res[10];
			value = res[25];
		}
		
		for (int i = 0; i < res.length; i++) {
			System.out.println("str " + i + "=" + res[i]);
		}
	}
	
	PShareEvent(byte [] contents, int len) throws PShareEventDecodingError {
		this.contents = contents;
		this.len = len;
		try {
			decodeMessage();
		} catch (UnsupportedEncodingException e) {
			throw new PShareEventDecodingError(e);
		}
	}
	
	public byte [] byteContents() {
		return contents;
	}

	public int byteLen() {
		return len;
	}
	
	public String toString() {
		return "<PShareEvent:" + DebugLog.bytesToHex(contents, len) + ">";
	}
}