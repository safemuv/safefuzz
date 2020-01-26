package atlasdsl;

import java.util.Optional;

public abstract class GoalAction {
	protected abstract Optional<GoalResult> test(GoalParticipants participants);
}
