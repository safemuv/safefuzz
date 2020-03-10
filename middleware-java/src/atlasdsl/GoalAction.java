package atlasdsl;

import java.util.Optional;

import middleware.core.ATLASCore;

public abstract class GoalAction {
	protected abstract Optional<GoalResult> test(Mission mission, GoalParticipants participants);
	protected abstract void setup(ATLASCore core, Mission mission, Goal g) throws GoalActionSetupFailure;
}
