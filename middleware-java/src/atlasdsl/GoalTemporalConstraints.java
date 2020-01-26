package atlasdsl;

import java.util.List;

public class GoalTemporalConstraints {
	private List<Goal> dependencies; 
	double startTime;
	double finishTime;
	
	public boolean isReady(double timeNow) {
		boolean depsReady = true;
		for (Goal d : dependencies) {
			if (!d.isReady(timeNow))
				depsReady = false;
		}
		return depsReady && (timeNow >= startTime) && (timeNow <= finishTime); 
	}

	public boolean isLate(double timeNow) {
		return (timeNow > finishTime);
	}
}