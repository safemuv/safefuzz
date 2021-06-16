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

	private int topNumToSelect = 3;
	private int sizeLimit;
	Random rng = new Random();
	
	public FuzzingPopulation(int sizeLimit) {
		this.sizeLimit = sizeLimit;
	}
	
	public Optional<FuzzingExptResult> pickPopulationElementToExplore() {
		// Select one in the top x% of the population...
		List<FuzzingExptResult> sortedPop = pop.stream().sorted(Comparator.reverseOrder()).collect(Collectors.toList());
		int elementSelection = rng.nextInt(Math.min(sortedPop.size(), topNumToSelect));
		FuzzingExptResult picked = sortedPop.get(elementSelection);
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
			if (sortedPop != null) {
				pop.remove(worst);
			}
		}
	}

	public int currentSize() {
		return pop.size();
	}
}
