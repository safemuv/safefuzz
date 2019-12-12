package moosmapping;

import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.util.List;

public class MOOSProcess extends MOOSElement {
	private List<MOOSBehaviour> behaviours;
	
	String processName; 
	
	private void generateBehaviourFile(MOOSFiles fs) {
		// Create a new file for the MOOS process behaviours
		// open it as file_stream
		
		FileOutputStream file_stream;
		
		try {
			
			file_stream = fs.createOpenFile(processName + ".bhv");
			for (MOOSBehaviour b : behaviours)
				b.generateCode(file_stream);			
			
		} catch (FileNotFoundException e) {
			e.printStackTrace();
		}
	}
	
	private void generateMissionFile(MOOSFiles fs) {
		// The process will generate a plugin in the core file?
	}
	
	public void generateCode(MOOSFiles fs) {
		generateMissionFile(fs);
		generateBehaviourFile(fs);
	}
}
