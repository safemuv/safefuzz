package atlasdsl;

import java.util.ArrayList;
import java.util.List;

public class GoalTemporalConstraints {
	private List<Goal> dependencies = new ArrayList<Goal>(); 
	double startTime;
	double finishTime;
	
	public GoalTemporalConstraints(double startTime, double finishTime) {
		this.startTime = startTime;
		this.finishTime = finishTime;
	}
	
	public void addDependency(Goal dependency) {
		dependencies.add(dependency);
	}
	
	public boolean isReady(double timeNow) {
		boolean depsReady = true;
		
		for (Goal d : dependencies) {
			if (!(d.isReady(timeNow)))
				depsReady = false;
		}
		return depsReady && (timeNow >= startTime) && (timeNow <= finishTime); 
	}

	public boolean isLate(double timeNow) {
		return (timeNow > finishTime);
	}

	public List<Goal> getDependencies() {
		return dependencies;
	}
}