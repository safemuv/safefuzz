package atlassharedclasses;

import java.util.Optional;

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
	
	public Optional<SonarDetection> getSonarDetection() {
		if (contentsClass == SonarDetection.class) {
			return Optional.of((SonarDetection)contents);
		} else return Optional.empty();
	}
}