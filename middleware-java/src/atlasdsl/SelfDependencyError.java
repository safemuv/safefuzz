package atlasdsl;

public class SelfDependencyError extends Exception {
	private static final long serialVersionUID = 1L;

	SelfDependencyError(Goal g1, Goal g2) {
	}
}