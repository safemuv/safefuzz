package faultgen;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.Scanner;
import atlasdsl.InvalidComponentType;
import atlasdsl.Mission;
import atlasdsl.faults.Fault;
import atlassharedclasses.FaultInstance;

public class FaultFileIO {
	private Mission mission;
	private CountHashmap<Fault> countFaults;
	
	public FaultFileIO(Mission mission) {
		this.mission = mission;
	}
	
	public Optional<FaultInstance> decodeFaultFromString(String faultDefinition) throws InvalidFaultFormat, InvalidComponentType, FaultNotFoundInModel, FaultInstanceInvalid, FaultRepeatCountInvalid {
		String[] fields = faultDefinition.split(",");
		//System.out.println("Fields length = " + fields.length + ", string=" + faultDefinition);
		if (fields.length < 3) {
			// Ignore the situation with an empty line or a single name tag
			if (fields.length > 1) {
				throw new InvalidFaultFormat();
			} else {
				return Optional.empty();
			}
		} else {
			
			int faultInstanceNum = Integer.parseInt(fields[0]);
			String faultNameInModel = fields[1];
			Double startTime = Double.parseDouble(fields[2]);
			Double length = Double.parseDouble(fields[3]);
			Double endTime = startTime + length;
			boolean isActive = Boolean.parseBoolean(fields[4]);
			Optional<String> extraData = Optional.empty();
			if (fields.length > 5) {
				extraData = Optional.of(fields[5]);
			}
			
			Optional<Fault> f_o = mission.lookupFaultByName(faultNameInModel);
			if (f_o.isPresent()) {
				Fault f = f_o.get();
				countFaults.incrementCount(f);
				if ((countFaults.getCount(f)) > f.getMaxCount()) {
					throw new FaultRepeatCountInvalid();
				}
				
				FaultInstance fi = new FaultInstance(startTime, endTime, f, extraData);
				fi.setActiveFlag(isActive);
				
				if (!fi.isValid()) {
					throw new FaultInstanceInvalid();
				} else return Optional.of(fi);
			} else {
				throw new FaultNotFoundInModel(faultNameInModel);
			}
		}
	}
	
	// TODO: should be renamed to loadFaultInstancesFromFile
	public List<FaultInstance> loadFaultsFromFile(String filename) throws FileNotFoundException {
		List<FaultInstance> outputFaults = new ArrayList<FaultInstance>();
		countFaults = new CountHashmap<Fault>();

		File f = new File(filename);
		Scanner reader = new Scanner(f);
		while (reader.hasNextLine()) {
			String faultAsString = reader.nextLine();
			Optional<FaultInstance> fi_o;
			
			try {
				fi_o = decodeFaultFromString(faultAsString);
				if (fi_o.isPresent()) {
					FaultInstance fi = fi_o.get();
					outputFaults.add(fi);
				}
				
			} catch (InvalidFaultFormat e) {
				e.printStackTrace();
			} catch (InvalidComponentType e) {
				e.printStackTrace();
			} catch (FaultNotFoundInModel e) {
				e.printStackTrace();
			} catch (FaultInstanceInvalid e) {
				e.printStackTrace();
			} catch (FaultRepeatCountInvalid e) {
				e.printStackTrace();
			}
		}
		reader.close();
		return outputFaults;
	}
}
