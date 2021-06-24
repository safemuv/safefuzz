package fuzzexperiment.runner;

import java.util.Comparator;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Random;
import java.util.Set;
import java.util.stream.Collectors;

public class FuzzingPopulation {
	public Set<FuzzingExptResult> pop = new HashSet<FuzzingExptResult>();

	private int topNumToSelect = 5;
	private int sizeLimit;
	Random rng = new Random();
	
	public FuzzingPopulation(int sizeLimit) {
		this.sizeLimit = sizeLimit;
	}
	
	private void printPop(List<FuzzingExptResult> res) {
		for (FuzzingExptResult r : res) {
			System.out.println(r.toString());
		}
	}
	
	public Optional<FuzzingExptResult> pickPopulationElementToExplore() {
		// Select one in the top x% of the population...
		List<FuzzingExptResult> sortedPop = pop.stream().sorted(Comparator.reverseOrder()).collect(Collectors.toList());
		// Print the population
		int elementSelection = rng.nextInt(Math.min(sortedPop.size(), topNumToSelect));
		FuzzingExptResult picked = sortedPop.get(elementSelection);
		System.out.println("pickPopulationElementToExplore - looking for highest");
		System.out.print("sortedPop=");
		printPop(sortedPop);
		System.out.println("elementSelection (from highest)=" + elementSelection);
		System.out.println("picked=" + picked);
		return Optional.of(picked);
	}

	public void pushToPopulation(FuzzingExptResult newResult) {
		// Add the new element to the population
		pop.add(newResult);
		// If the population size is greater than the size limit...
		if (pop.size() > sizeLimit) {
			// Sort and find one to evict ... the worst one
			List<FuzzingExptResult> sortedPop = pop.stream().sorted().collect(Collectors.toList());
			FuzzingExptResult worst = sortedPop.get(0);
			// TODO: ensure this sort order is correct
			if (worst != null) {
				pop.remove(worst);
			}
			
			System.out.println("pushToPopulation - looking for lowest");
			System.out.println("sortedPop=" + sortedPop);
			System.out.println("worst=" + worst);
		}
	}

	public int currentSize() {
		return pop.size();
	}
}
