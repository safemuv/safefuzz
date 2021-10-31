package utils;

import java.io.File;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;

public class DirectoryUtils {
	public static void ensureTempDirClear(String d) {
		File dirF = new File(d);
		Path dir = Paths.get(d);
		
		if (Files.exists(dir)) {
			for(File file: dirF.listFiles()) 
			    if (!file.isDirectory()) 
			        file.delete();
		} else {
			dirF.mkdir();
		}
	}
}
