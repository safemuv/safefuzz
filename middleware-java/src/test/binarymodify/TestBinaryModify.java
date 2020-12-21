package test.binarymodify;

import java.io.IOException;
import java.util.LinkedHashMap;
import java.util.Map;
import utils.binarymodify.BinaryModify;
import utils.binarymodify.IncorrectStringLength;

public class TestBinaryModify {
	public static void main(String [] args) {
		Map<String,String> m = new LinkedHashMap<String,String>();
		m.put("DESIRED_RUDDER", "DEZIRED_RUDDER");
		m.put("DESIRED_THRUST", "DEZIRED_THRUST");
		
		try {
			BinaryModify.BBEModifyFile("/tmp/uSimMarine", "/tmp/uSimMarine_f", m);
		} catch (IncorrectStringLength e) {
			e.printStackTrace();
		} catch (IOException e) {
			e.printStackTrace();
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
	}
}