package moosmapping;

import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

public class MOOSFiles {
	private String baseDirectory;
	private HashMap<String,FileWriter> files = new HashMap<String,FileWriter>();
	
	public MOOSFiles(String baseDirectory) {
		this.baseDirectory = baseDirectory;
	}
	
	public FileWriter createOpenFile(String fileName) throws IOException {
		String filePathName = baseDirectory + "/" + fileName;
		FileWriter f = new FileWriter(filePathName);
		f.write("Test");
		f.flush();
		files.put(fileName, f);
		return f;
	}
	
	public void closeAllFiles() {
		for (FileWriter f : files.values()) {
		    try {
				f.close();
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
	}
}
