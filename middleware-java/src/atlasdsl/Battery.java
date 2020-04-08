package atlasdsl;

public class Battery extends EnergyResource {
	private int currentEnergy;
	private int maxEnergy;
	
	public Battery(int startingEnergy) {
		this.currentEnergy = startingEnergy;
		this.maxEnergy = startingEnergy;
	}
	
	public int getEnergyRemaining() {
		return currentEnergy;
	}
	
	public double getPercentageRemaining() {
		return ((double)currentEnergy) / ((double)maxEnergy);
	}
	
	public boolean isExhausted() {
		return (currentEnergy > 0);
	}
	
	public void depleteEnergy(int depletionEnergy) {
		currentEnergy -= depletionEnergy;
		if (currentEnergy < 0) 
			currentEnergy = 0;
	}
}
