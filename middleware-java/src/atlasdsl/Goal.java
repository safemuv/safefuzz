package atlasdsl;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;

import fuzzexperiment.runner.metrics.Metric;
import middleware.core.ATLASCore;

public class Goal {
	private String name;
	protected Mission mission;
	private GoalTemporalConstraints timingReqs;
	private GoalParticipants participants;
	private Optional<GoalRegion> region;
	private GoalAction action;
	private List<GoalResult> results = new ArrayList<GoalResult>();
	private List<GoalVariable> variables = new ArrayList<GoalVariable>();
	private Set<Metric> metrics = new HashSet<Metric>();
	
	public Goal(String name, Mission mission, GoalTemporalConstraints timingReqs, GoalParticipants participants, Optional<GoalRegion> region, GoalAction action) {
		this.name = name;
		this.mission = mission;
		this.timingReqs = timingReqs;
		this.participants = participants;
		this.region = region;
		this.action = action;
	}
	
	private GoalStatus status = GoalStatus.PENDING;
	
	public boolean isReady(double timeNow) {
		System.out.println("isReady on " + name);
		return timingReqs.isReady(timeNow);
	}
	
	public GoalStatus getStatus() {
		return status;
	}
	
	public String getName() {
		return name;
	}
	
	public void setStatus(GoalStatus gs) {
		status = gs;
	} 

	public boolean isLate(double timeNow) {
		return timingReqs.isLate(timeNow);
	}
	
	public Optional<GoalResult> test() {
		return action.test(mission, this.participants);
	}
	
	public void setup(ATLASCore core) throws GoalActionSetupFailure {
		action.setup(core, mission, this);
	}

	public Optional<GoalRegion> getGoalRegion() {
		return region;
	}

	public void registerResult(GoalResult gr) {
		results.add(gr);
	}
	
	public List<GoalResult> getResults() {
		return results;
	}
	
	public GoalAction getAction() {
		return action;
	}
	
	public void setDependencyOn(Goal dep) throws SelfDependencyError {
		if (this == dep) {
			throw new SelfDependencyError(this, dep);
		} else {
			System.out.println("setting dependency from " + this.name + " on " + dep.name);
			this.timingReqs.addDependency(dep);
		}
	}

	public boolean allDependenciesComplete() {
		List<Goal> deps = timingReqs.getDependencies();
		boolean complete = true;
		for (Goal g : deps) { 
			if (!(g.status == GoalStatus.COMPLETED)) {
				complete = false;
			}
		}
		return complete;
	}
	
	public void addVariable(GoalVariable gv) {
		variables.add(gv);
	}
	
	public List<GoalVariable> getGoalVariables() {
		return variables;
	}
	
	public Set<Metric> getAllMetrics() {
		return metrics;
	}

	public void addMetric(Metric met) {
		metrics.add(met);
	}
}
