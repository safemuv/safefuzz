package test;

import java.io.IOException;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import com.fasterxml.jackson.databind.ObjectMapper;

public class TestCode {
	

	
	private static Pattern nodeReportScanner = Pattern.compile("X=([^,]+),Y=([^,]+)");
	
	public static void testMatch(String test) {
		Matcher m = nodeReportScanner.matcher(test);
		if (m.find()) {
			double x = Double.parseDouble(m.group(1));
			double y = Double.parseDouble(m.group(2));
			System.out.println("x = " + x + " : y = " + y);
		} else {
			System.out.println("Match failed!");
		}
	}
	
	public static void testSerialise() {
		ObjectMapper objectMapper = new ObjectMapper();
		String carJson =
			    "{ \"brand\" : \"Mercedes\", \"doors\" : 5 }";
			try {
			    Car car = objectMapper.readValue(carJson, Car.class);
			    Car car2 = new Car("Ford", 4);
			    String car2Serialised = objectMapper.writeValueAsString(car2);
			    
			    System.out.println("contents = " + car2Serialised);
			    
			    System.out.println("car brand = " + car.getBrand());
			    System.out.println("car doors = " + car.getDoors());
			} catch (IOException e) {
			    e.printStackTrace();
			}

		//JaxbAnnotationModule jaxbAnnotationModule = new JaxbAnnotationModule();
		//XmlMapper mapper = new XmlMapper();
		//mapper.enable(SerializationFeature.INDENT_OUTPUT);
		//mapper.registerModule(jaxbAnnotationModule);
		//mapper.registerModule(new GuavaModule());
		//String xml = mapper.writeValueAsString(customer);
		//System.out.println(xml);
	}
	
	public static void main(String args []) {
		testMatch("X=32.52,Y=-3.132");
		testMatch("X=21,Y=2");
		testMatch("X=2,Y=7,Z=532");
		testMatch("T=121,X=32.52,Y=-4.21");
		testMatch("T=121,X=122.,Y=-4.21");
		
		testSerialise();
	}
}