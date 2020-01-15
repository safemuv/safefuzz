package atlascollectiveintgenerator;

import atlasdsl.*;

public abstract class CollectiveIntGen {
	protected Mission mission;
	
	public CollectiveIntGen(Mission m) {
		this.mission = m;
	}
	
	public abstract void generateCollectiveIntFiles(String baseDir, CollectiveIntGenTypes cg);
}
