package atlasdsl;

import atlassharedclasses.*;

public class PositionTracker {
	private Region region;
	private int countRequired;

	private int counts[][];
	private boolean visitedEver[][];
	private int uniqueCount = 0;
	private int totalElements;

	final int X_WIDTH = 500;
	final int Y_HEIGHT = 500;
	int xsize, ysize;

	private Point basePoint = new Point(0, 0);

	// Need to setup a 2D array which tracks the instance of visits to
	// a particular point
	// TODO: this will currently work for a 2D region, but an irregular
	// polygon will need some kind of mask in order to track it?

	// This point tracker needs to be displayed in the GUI
	public PositionTracker(Region region, int countRequired) {
		final int X_WIDTH = 5;
		final int Y_HEIGHT = 5;

		this.region = region;
		this.countRequired = countRequired;

		xsize = (int) (region.width() / X_WIDTH);
		ysize = (int) (region.height() / Y_HEIGHT);
		this.counts = new int[xsize][ysize];
		this.visitedEver = new boolean[xsize][ysize];
		this.totalElements = xsize * ysize;
		this.uniqueCount = 0;
	}

	public synchronized void notifyCoordinate(Point coordinate) {
		// Work out the coordinates for the point tracker here
		Point diff = coordinate.sub(basePoint);
		int x = Math.floorDiv((int) diff.getX(), X_WIDTH);
		int y = Math.floorDiv((int) diff.getY(), Y_HEIGHT);

		System.out.println("x=" + x + "," + "y=" + y);
		if (x > 0 && y > 0 && x < xsize && y < ysize) {
			counts[x][y] += 1;
			if (!visitedEver[x][y]) {
				visitedEver[x][y] = true;
				uniqueCount++;
			}
		}
	}

	// Indicates when all the area is complete
	public boolean isComplete() {
		return (uniqueCount == totalElements);
	}
}
