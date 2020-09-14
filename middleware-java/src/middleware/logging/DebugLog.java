package middleware.logging;

import java.nio.charset.StandardCharsets;

public class DebugLog {
	
	private static final byte[] HEX_ARRAY = "0123456789ABCDEF".getBytes(StandardCharsets.US_ASCII);
	public static String bytesToHex(byte[] bytes, int len) {
	    byte[] hexChars = new byte[len * 2];
	    for (int j = 0; j < len; j++) {
	        int v = bytes[j] & 0xFF;
	        hexChars[j * 2] = HEX_ARRAY[v >>> 4];
	        hexChars[j * 2 + 1] = HEX_ARRAY[v & 0x0F];
	    }
	    return new String(hexChars, StandardCharsets.UTF_8);
	}
	
    // A utility method to convert the byte array 
    // data into a string representation. 
    public static StringBuilder debugString(byte[] a, int len) { 
        if (a == null) 
            return null; 
        StringBuilder ret = new StringBuilder(); 
        int i = 0; 
        while (a[i] != 0 && i < len) 
        { 
            ret.append((char)a[i]); 
            i++; 
        } 
        return ret; 
    } 
}
