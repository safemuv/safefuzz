package middleware.gui;

import java.awt.*;
import java.awt.image.MemoryImageSource;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Arrays;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.Map;

import javax.swing.*;

import atlasdsl.Goal;
import atlasdsl.GoalAction;
import atlasdsl.PositionTracker;
import atlasdsl.SensorCover;
import atlassharedclasses.Region;

public class PositionTrackingOutput extends JPanel {
	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;
	HashMap<Region,Image> regionImages = new LinkedHashMap<Region, Image>();
	GUITest gui;
	
	public PositionTrackingOutput(GUITest gui) {
		this.gui = gui;
		this.setSize(100,100);
	}
	
	public void logCounts(Region r, int[][] counts, int x, int y) {
		String filename = "region-pt-debug-" + r.hashCode();
		try {
			FileWriter fw = new FileWriter(filename);
			for (int i = 0; i < x; i++) {
				for (int j = 0; j < y; j++) {
					fw.append(counts[i][j] + "  "); 
				}
				fw.append("\n");
			}
			fw.close();
		} catch (IOException e) {
			e.printStackTrace();
		}
	}
	
	public void paint(Graphics g) {
		System.out.println("paint");
		Graphics2D g2d = (Graphics2D)g;
		g.drawRect(0, 0, 100, 100);
		g.drawLine(0, 0, 100, 100);
		
    	for (Map.Entry<Region, Image> me :regionImages.entrySet()) {
    		System.out.println("Image");
    		Image img = me.getValue();
    		System.out.println(img.getWidth(null) + "-" + img.getHeight(null));
    		g2d.drawImage(img, 0, 0, null);
    	}
	}	
	
    private void updateRegionImage(Region r, Image img) {
    	regionImages.put(r, img);
    	System.out.println("updateRegionImage - count " + regionImages.size());
    }
    
    private void renderPositionTracker(Region r, PositionTracker pt) {
    	int x = pt.getXSize();
    	int y = pt.getYSize();
    	int[][] counts = pt.getCounts();
    	
    	// Convert the 2D pixel array into 1D
    	int[] pixels = Arrays.stream(counts)
    	        .flatMapToInt(Arrays::stream)
    	        .toArray();
    	
    	logCounts(r, counts, x, y);
    	
    	MemoryImageSource ms = new MemoryImageSource(x,y,pixels,0,x);
    	Image img = Toolkit.getDefaultToolkit().createImage(ms);
    	updateRegionImage(r, img);
    }
    
    private void renderCollectiveSensorCover(SensorCover csc) {
    	Map<Region,PositionTracker> pts = csc.getPosTrackers();
    	for (Map.Entry<Region,PositionTracker> me : pts.entrySet()) {
    		Region r = me.getKey();
    		PositionTracker pt = me.getValue(); 
    		renderPositionTracker(r,pt);
    	}
    }
    
    public void updateGoalInfo() {
    	System.out.println("updateGoalInfo");
    	HashMap<Goal,JLabel> goalLabels = gui.getGoalLabels();
    	
    	for (Goal g : goalLabels.keySet()) {
    		GoalAction ga = g.getAction();
    		if (ga instanceof SensorCover) {
    			SensorCover csc = (SensorCover)ga;
    	   		// TODO: This call will likely be slow... may have to only do it
        		// periodically, e.g. 1/10th of calls 
    			renderCollectiveSensorCover(csc);
    		}
    	}
    }
}
