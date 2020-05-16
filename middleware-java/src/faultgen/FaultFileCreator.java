package faultgen;

import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import atlasdsl.*;
import atlasdsl.faults.*;
import atlassharedclasses.FaultInstance;

public class FaultFileCreator {
	private Mission mission;
	
	int faultsToAdd = 0;
	int faultsToRemove = 0;
	
	private String baseFaultDirectory;
	private Random random;
		
	public FaultFileCreator(Mission mission, String baseFaultDirectory) {
		this.mission = mission;
		this.baseFaultDirectory = baseFaultDirectory;
		// TODO: seeds for random generator
		this.random = new Random();
	}
	
	public void generateFile(String outputFile) {
	// Generate potential faults according to the mission definition
	// Mutate an existing file?
		
	// If there are no faults in the system - create entirely new ones from the 
	List<Fault> faults = mission.getFaultsAsList();
	}
	
	// TODO: factor out fault writing/reading code from FaultGenerator into a common utility class for 
	// file IO/format reading
	public void writeFaultInstance(int faultInstanceCount, FileWriter fw, FaultInstance fi) throws IOException {
		Fault f = fi.getFault();
		String output = Integer.toString(faultInstanceCount) + "," + f.getName() +  "," +  Double.toString(fi.getStartTime()) + "," + Double.toString(fi.getEndTime()) + "," + fi.getExtraData() + "\n";
		fw.append(output);
	}
	
	public void writeFaultDefinitionFile(String outFile, List<FaultInstance> outputFaultInstances) throws IOException {
		// Iterate over the list of output faults and write them out
		FileWriter fw = new FileWriter(outFile);
		int fiCount = 0;
		for (FaultInstance fi : outputFaultInstances) {
			fiCount++;
			writeFaultInstance(fiCount, fw, fi);
		}
		fw.close();
	}
	
	public void generateFaultListFromScratch(String outputFile) {
		List<FaultInstance> outputFaultInstances = new ArrayList<FaultInstance>();
		List<Fault> faults = mission.getFaultsAsList();
		for (Fault f : faults) {
			int count = f.getMaxCount();
			// Create count faults in the time range
			for (int i = 0; i < count; i++) {
				FaultTimeProperties ftp = f.getTimeProperties();
				// TODO: check this computation of faults
				if (random.nextDouble() < ftp.getFaultProb()) {
					// TODO: argument RANDOM_UNIFORM is not yet used in any way - all generation is uniform
					FaultInstance fi = f.randomWithin(FaultCreationTypes.RANDOM_UNIFORM);
					outputFaultInstances.add(fi);
				}
			}
		}
		
		try {
			writeFaultDefinitionFile(baseFaultDirectory + "/" + outputFile, outputFaultInstances);
		} catch (IOException e) {
			e.printStackTrace();
		}
	}
	
	public void mutateFaults(String inputFile, String outputFile) {
		List<FaultInstance> outputFaultInstances = new ArrayList<FaultInstance>();
		List<Fault> faults = mission.getFaultsAsList();
		for (Fault f : faults) {
			int count = f.getMaxCount();
			// Create count faults in the time range
			for (int i = 0; i < count; i++) {
				FaultTimeProperties ftp = f.getTimeProperties();
				// TODO: check this computation of faults
				if (random.nextDouble() < ftp.getFaultProb()) {
					// TODO: argument RANDOM_UNIFORM is not yet used in any way - all generation is uniform
					FaultInstance fi = f.randomWithin(FaultCreationTypes.RANDOM_UNIFORM);
					outputFaultInstances.add(fi);
				}
			}
		}
		
		try {
			writeFaultDefinitionFile(baseFaultDirectory + "/" + outputFile, outputFaultInstances);
		} catch (IOException e) {
			e.printStackTrace();
		}
	}
	
}
