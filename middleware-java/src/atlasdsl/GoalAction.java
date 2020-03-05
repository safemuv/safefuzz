package atlasdsl;

import java.util.Optional;

public abstract class GoalAction {
	protected abstract Optional<GoalResult> test(Mission mission, GoalParticipants participants);
	protected abstract void setup(Mission mission, Goal g);
}
