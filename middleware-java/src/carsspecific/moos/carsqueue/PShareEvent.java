package carsspecific.moos.carsqueue;

import java.io.UnsupportedEncodingException;
import java.util.Arrays;

import middleware.core.CARSEvent;
import middleware.logging.DebugLog;

public class PShareEvent extends CARSEvent {
	
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
	
	private PShareEvent() {
		
	}
	
	private static PShareEvent clone(PShareEvent e) {
		PShareEvent newE = new PShareEvent();
		newE.contents = e.contents;
		newE.len = e.len;
		newE.componentName = e.componentName;
		newE.communityName = e.communityName;
		newE.key = e.key;
		newE.value = e.value;
		return newE;
	}
	
	public byte [] byteContents() {
		return contents;
	}

	public int byteLen() {
		return len;
	}
	
	public String getKey() {
		return key;
	}
	
	public String getValue() {
		return value;
	}
	
	public String toString() {
		return "<PShareEvent:" + DebugLog.bytesToHex(contents, len) + ">";
	}

	public PShareEvent cloneWithNewValue(String key, String newValue) throws PShareEventDecodingError {
		PShareEvent eNew = PShareEvent.clone(this);
		eNew.value = newValue;
		// replace the string in the contents
		return eNew;
	}
}