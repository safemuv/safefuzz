package atlassharedclasses;

import java.util.Optional;

// TODO: This class is meant as a sort-of typesafe holder
// of serialised objects. It should at least be renamed and 
// probably re-engineered into a better solution
public class ATLASSharedResult {
	private Object contents;
	private Class<?> contentsClass;
	
	public ATLASSharedResult(Object contents, Class<?> contentsClass) {
		this.contents = contents;
		this.contentsClass = contentsClass;
	}
	
	public Object getContents() {
		return contents;
	}
	
	public Class<?> getContentsClass() {
		return contentsClass;
	}
	
	public Optional<GPSPositionReading> getGPSPositionReading() {
		if (contentsClass == GPSPositionReading.class) {
			return Optional.of((GPSPositionReading)contents);
		} else return Optional.empty();
	}
	
	// TODO: remove SonarDetection when the sensor types are fully
	// generalised, replace with SensorDetection
	//public Optional<SonarDetection> getSonarDetection() {
//		if (contentsClass == SonarDetection.class) {
			//return Optional.of((SonarDetection)contents);
		//} else return Optional.empty();
	//}
	
	public Optional<SensorDetection> getSensorDetection() {
		if (contentsClass == SensorDetection.class) {
			return Optional.of((SensorDetection)contents);
		} else return Optional.empty();
	}
	
	public Optional<CIEvent> getCIEvent() {
		if (contentsClass == CIEvent.class) {
			return Optional.of((CIEvent)contents);
		} else return Optional.empty();
	}
	
	public Optional<ATLASTimeUpdate> getATLASTimeUpdate() {
		if (contentsClass == ATLASTimeUpdate.class) {
			return Optional.of((ATLASTimeUpdate)contents);
		} else return Optional.empty();
	}
}