package atlasdsl.loader;

import atlasdsl.*;
import atlassharedclasses.*;

public class GeneratedDSLLoader implements DSLLoader {
	public Mission loadMission() throws DSLLoadFailed {
		final double MISSION_END_TIME = 1200.0;
		final boolean STOP_ON_NO_ENERGY = true;
		Mission mission = new Mission(MISSION_END_TIME, STOP_ON_NO_ENERGY);

		Computer c1 = new Computer("shoreside");
		mission.addComputer(c1);

		Robot r1 = new Robot("uav_1");
		r1.setPointComponentProperty("startLocation", new Point(10.0, 10.0, 0.0));
		mission.addRobot(r1);

		Robot r2 = new Robot("uav_2");
		r2.setPointComponentProperty("startLocation", new Point(0.0, 10.0, 0.0));
		mission.addRobot(r2);
		return mission;
	}
}