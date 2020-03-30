package atlasdsl.loader;

import atlasdsl.*;

public interface DSLLoader {
	public Mission loadMission() throws DSLLoadFailed;
}
