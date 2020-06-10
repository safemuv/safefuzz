package test.serialisation;

import java.util.Optional;

import com.fasterxml.jackson.core.JsonProcessingException;

import atlasdsl.*;
import atlassharedclasses.*;
import atlassharedclasses.ATLASObjectMapper.ATLASFormatError;

public class TestSensorDetections {
	
	public static void testSerialisation() {
		ATLASObjectMapper ao = new ATLASObjectMapper();
		
		Message m = new Message("TEST");
		SensorDetection dIn = new SensorDetection(m, SensorType.CAMERA);
		System.out.println("dIn = " + dIn.toString());
		try {
			
			dIn.setField("THING1", "Hello");
			dIn.setField("THING2", 53.6);
			String msg = ao.serialise(dIn);
			
			System.out.println("Serialised string: " + msg);
			ATLASSharedResult res = ao.deserialise(msg);
			Optional<SensorDetection> dOut_opt = res.getSensorDetection();
			if (dOut_opt.isPresent()) {
				SensorDetection dOut = dOut_opt.get();
				System.out.println("dOut type = " + dOut.getSensorType());
				System.out.println("dOut = " + dOut.toString());
				
			}
			
		} catch (JsonProcessingException e) {
			e.printStackTrace();
		} catch (ATLASFormatError e) {
			e.printStackTrace();
		}
	}
	
	
	public static void main(String [] args) {
		testSerialisation();
	}
}
