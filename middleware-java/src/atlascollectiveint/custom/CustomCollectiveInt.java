package atlascollectiveint.custom;

import atlassharedclasses.*;
import atlasdsl.SensorType;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.Optional;

import atlascollectiveintgenerator.CollectiveInt;
import atlassharedclasses.ATLASSharedResult;

public class CustomCollectiveInt extends CollectiveInt {
	Method sonarMethod;
	Method gpsMethod;
	Method cameraMethod;
	Method energyMethod;
	Method initMethod;

	public CustomCollectiveInt(String cIClassName)
			throws ClassNotFoundException, InstantiationException, IllegalAccessException, IllegalArgumentException,
			InvocationTargetException, NoSuchMethodException, SecurityException {
		super(cIClassName);

		System.out.println("Looking up hook methods on CI class " + ciClass.getName());
		sonarMethod = ciClass.getDeclaredMethod("SONARDetectionHook", SensorDetection.class, String.class);
		gpsMethod = ciClass.getDeclaredMethod("GPS_POSITIONDetectionHook", Double.class, Double.class, String.class);
		cameraMethod = ciClass.getDeclaredMethod("CAMERADetectionHook", SensorDetection.class, String.class);
		energyMethod = ciClass.getDeclaredMethod("EnergyUpdateHook", EnergyUpdate.class, String.class);
		initMethod = ciClass.getDeclaredMethod("init");
		System.out.println("Looked up all methods on CI class successfully");
	}

	protected void handleMessage(ATLASSharedResult a) {
		super.handleMessage(a);

		try {

			if (a.getContentsClass() == SensorDetection.class) {
				Optional<SensorDetection> d_o = a.getSensorDetection();
				if (d_o.isPresent()) {
					SensorDetection d = d_o.get();
					if (d.getSensorType() == SensorType.SONAR) {
						sonarMethod.invoke(null, d_o.get(), d.getField("robotName"));
					}
				}
			};

			if (a.getContentsClass() == GPSPositionReading.class) {
				Optional<GPSPositionReading> r_o = a.getGPSPositionReading();
				if (r_o.isPresent()) {
					GPSPositionReading r = r_o.get();
					gpsMethod.invoke(null, r.getX(), r.getY(), r.getRobotName());
				}
			}

			if (a.getContentsClass() == SensorDetection.class) {
				Optional<SensorDetection> d_o = a.getSensorDetection();
				if (d_o.isPresent()) {
					SensorDetection d = d_o.get();
					if (d.getSensorType() == SensorType.CAMERA) {
						cameraMethod.invoke(null, d_o.get(), d.getField("robotName"));
					}
				}
			};
			
			if (a.getContentsClass() == EnergyUpdate.class) {
				Optional<EnergyUpdate> e_o = a.getEnergyUpdate();
				if (e_o.isPresent()) {
					EnergyUpdate e = e_o.get();
					energyMethod.invoke(null, e_o.get(), e.getRobotName());
				}
			}
			
		} catch (InvocationTargetException e) {
			e.printStackTrace();
		} catch (IllegalAccessException e) {
			e.printStackTrace();
		} catch (IllegalArgumentException e) {
			e.printStackTrace();
		}
	}

	public void init() {
		super.init();
		try {
			initMethod.invoke(null);
		} catch (IllegalAccessException | IllegalArgumentException | InvocationTargetException e) {
			e.printStackTrace();
		}
	}
}
