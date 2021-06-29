package fuzzexperiment.runner;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashSet;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.Random;
import java.util.Set;
import java.util.stream.Collectors;

public class FuzzingPopulation {
	public List<FuzzingExptResult> pop = new ArrayList<FuzzingExptResult>();
	


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
	
	public void print() {
		printPop(pop);
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
	
	//https://github.com/jMetal/jMetal/blob/9a68b6878a1d7429fb1a0be87f84ec6faa05fa8c/jmetal-core/src/main/java/org/uma/jmetal/util/ranking/impl/FastNonDominatedSortRanking.java
	public List<ArrayList<FuzzingExptResult>> computeRankings() {
		List<ArrayList<FuzzingExptResult>> rankedSubPopulations;
	    List<FuzzingExptResult> population = new ArrayList<FuzzingExptResult>(pop);
	    

	    // dominateMe[i] contains the number of population dominating i
	    int[] dominateMe = new int[population.size()];

	    // iDominate[k] contains the list of population dominated by k
	    List<List<Integer>> iDominate = new ArrayList<>(population.size());

	    // front[i] contains the list of individuals belonging to the front i
	    ArrayList<List<Integer>> front = new ArrayList<>(population.size() + 1);

	    // Initialize the fronts
	    for (int i = 0; i < population.size() + 1; i++) {
	      front.add(new LinkedList<Integer>());
	    }

	    // Fast non dominated sorting algorithm
	    // Contribution of Guillaume Jacquenot
	    for (int p = 0; p < population.size(); p++) {
	      // Initialize the list of individuals that i dominate and the number
	      // of individuals that dominate me
	      iDominate.add(new LinkedList<Integer>());
	      dominateMe[p] = 0;
	    }
	    
	    
	    

	    int flagDominate = 0;
	    for (int p = 0; p < (population.size() - 1); p++) {
	      // For all q individuals , calculate if p dominates q or vice versa
	      for (int q = p + 1; q < population.size(); q++) {
	        //flagDominate =
	        //   CONSTRAINT_VIOLATION_COMPARATOR.compare(population.get(p), population.get(q));
	        //if (flagDominate == 0) {
	          //flagDominate = dominanceComparator.compare(population.get(p), population.get(q));
	          flagDominate = population.get(p).compareTo(population.get(q));
	          System.out.println(flagDominate);
	        //}
	        if (flagDominate == -1) {
	          iDominate.get(p).add(q);
	          dominateMe[q]++;
	        } else if (flagDominate == 1) {
	          iDominate.get(q).add(p);
	          dominateMe[p]++;
	        }
	      }
	    }
	    
	    for (int i = 0; i < population.size(); i++) {
	        if (dominateMe[i] == 0) {
	          front.get(0).add(i);
	        }
	    }
	    
	    // Obtain the rest of fronts
	    int i = 0;
	    Iterator<Integer> it1, it2; // Iterators
	    while (front.get(i).size() != 0) {
	      i++;
	      it1 = front.get(i - 1).iterator();
	      while (it1.hasNext()) {
	        it2 = iDominate.get(it1.next()).iterator();
	        while (it2.hasNext()) {
	          int index = it2.next();
	          dominateMe[index]--;
	          if (dominateMe[index] == 0) {
	            front.get(i).add(index);
	          }
	        }
	      }
	    }

	    rankedSubPopulations = new ArrayList<>();
	    // 0,1,2,....,i-1 are fronts, then i fronts
	    for (int j = 0; j < i; j++) {
	      rankedSubPopulations.add(j, new ArrayList<FuzzingExptResult>(front.get(j).size()));
	      it1 = front.get(j).iterator();
	      while (it1.hasNext()) {
	        rankedSubPopulations.get(j).add(population.get(it1.next()));
	      }
	    }
	    
	    return rankedSubPopulations;
	}
}
