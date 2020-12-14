package utils.binarymodify;

import java.io.IOException;
import java.util.Map;

public class BinaryModify {
	private static void chmodFileToExecutable(String fullPath) throws IOException, InterruptedException {
		ProcessBuilder processBuilder = new ProcessBuilder();
		processBuilder.command("bash", "-c", "chmod 755 " + fullPath);
		Process process = processBuilder.start();
		int ret = process.waitFor();
		System.out.println("Chmodding file to executable");
	}
	
	public static int BBEModifyFile(String fullFilePath, String outputFullPath, Map<String,String> fromTo) throws IncorrectStringLength, IOException, InterruptedException {
		ProcessBuilder processBuilder = new ProcessBuilder();
		System.out.println("Transforming file " + fullFilePath + " -> " + outputFullPath + " - " + fromTo.size() + " entries to modify");
		
		String cmd = "bbe ";
		for (Map.Entry<String,String> me : fromTo.entrySet()) {
			String inVar = me.getKey();
			String outVar = me.getValue();
						
			if (inVar.length() == outVar.length()) {
				cmd += "-e \"s/" + inVar + "/" + outVar + "/\" ";
			} else {
				throw new IncorrectStringLength(inVar, outVar);
			}
		}
		
		cmd += " " + fullFilePath + " > " + outputFullPath;
		System.out.println(cmd);
		processBuilder.command("bash", "-c",  cmd);
		Process process = processBuilder.start();
		int ret = process.waitFor();
		chmodFileToExecutable(outputFullPath);
		System.out.println("Transformation complete!");
		return ret;
	}
}
