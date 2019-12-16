package moosmapping;

import java.io.FileWriter;
import java.io.IOException;

public class MOOSBehaviour extends MOOSElement {
	private String name;
	private MOOSProcess parentProcess;
	
	public void generateCode(FileWriter fs) {
		try {
			fs.write("Behaviour = " + name);
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
}
