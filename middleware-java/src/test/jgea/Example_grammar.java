package test.jgea;

/*
 * Copyright 2020 Eric Medvet <eric.medvet@gmail.com> (as eric)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

import com.google.common.collect.Range;
import it.units.malelab.jgea.*;
import it.units.malelab.jgea.core.Individual;
import it.units.malelab.jgea.core.Problem;
import it.units.malelab.jgea.core.evolver.*;
import it.units.malelab.jgea.core.evolver.stopcondition.Iterations;
import it.units.malelab.jgea.core.evolver.stopcondition.TargetFitness;
import it.units.malelab.jgea.core.fitness.BooleanFunctionFitness;
import it.units.malelab.jgea.core.listener.*;
import it.units.malelab.jgea.core.listener.telegram.TelegramUpdater;
import it.units.malelab.jgea.core.order.ParetoDominance;
import it.units.malelab.jgea.core.order.PartialComparator;
import it.units.malelab.jgea.core.selector.Tournament;
import it.units.malelab.jgea.core.selector.Worst;
import it.units.malelab.jgea.core.util.ImagePlotters;
import it.units.malelab.jgea.core.util.Misc;
import it.units.malelab.jgea.core.util.Sized;
import it.units.malelab.jgea.problem.booleanfunction.BooleanUtils;
import it.units.malelab.jgea.problem.booleanfunction.EvenParity;
import it.units.malelab.jgea.problem.booleanfunction.FormulaMapper;
import it.units.malelab.jgea.problem.booleanfunction.EvenParity.TargetFunction;
import it.units.malelab.jgea.problem.booleanfunction.element.Element;
import it.units.malelab.jgea.problem.symbolicregression.*;
import it.units.malelab.jgea.problem.synthetic.LinearPoints;
import it.units.malelab.jgea.problem.synthetic.OneMax;
import it.units.malelab.jgea.problem.synthetic.Rastrigin;
import it.units.malelab.jgea.problem.synthetic.Sphere;
import it.units.malelab.jgea.representation.grammar.Grammar;
import it.units.malelab.jgea.representation.grammar.GrammarBasedProblem;
import it.units.malelab.jgea.representation.grammar.cfggp.GrammarBasedSubtreeMutation;
import it.units.malelab.jgea.representation.grammar.cfggp.GrammarRampedHalfAndHalf;
import it.units.malelab.jgea.representation.sequence.FixedLengthListFactory;
import it.units.malelab.jgea.representation.sequence.UniformCrossover;
import it.units.malelab.jgea.representation.sequence.bit.BitFlipMutation;
import it.units.malelab.jgea.representation.sequence.bit.BitString;
import it.units.malelab.jgea.representation.sequence.bit.BitStringFactory;
import it.units.malelab.jgea.representation.sequence.numeric.GaussianMutation;
import it.units.malelab.jgea.representation.sequence.numeric.GeometricCrossover;
import it.units.malelab.jgea.representation.sequence.numeric.UniformDoubleFactory;
import it.units.malelab.jgea.representation.tree.SameRootSubtreeCrossover;
import it.units.malelab.jgea.representation.tree.Tree;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.concurrent.ExecutionException;
import java.util.function.Function;
import java.util.stream.Collectors;

import static it.units.malelab.jgea.core.listener.NamedFunctions.*;

/**
 * @author eric
 */
public class Example_grammar extends Worker {

  public final static List<NamedFunction<Event<?, ?, ?>, ?>> BASIC_FUNCTIONS = List.of(
      iterations(),
      births(),
      elapsedSeconds(),
      size().of(all()),
      size().of(firsts()),
      size().of(lasts()),
      uniqueness().of(each(genotype())).of(all()),
      uniqueness().of(each(solution())).of(all()),
      uniqueness().of(each(fitness())).of(all()),
      size().of(genotype()).of(best()),
      size().of(solution()).of(best()),
      birthIteration().of(best())
  );

  public final static List<NamedFunction<Event<?, ?, ? extends Double>, ?>> DOUBLE_FUNCTIONS = List.of(
      fitness().reformat("%5.3f").of(best()),
      hist(8).of(each(fitness())).of(all()),
      max(Double::compare).reformat("%5.3f").of(each(fitness())).of(all())
  );

  public Example_grammar(String[] args) throws FileNotFoundException {
    super(args);
  }

  public static void main(String[] args) throws FileNotFoundException {
    new Example_grammar(args);
  }

  @Override
  public void run() {
    runGrammarBasedParity();
  }

  // Grammar:
  // cond_comp :== cond binop cond | unop cond | cond
  // cond :== expr comp expr | cond_simple
  // cond_simple :== var comp expr
  // expr :== expr c_binop expr | func expr | var | const
  
  // c_binop :== EQUALS | NOT_EQUALS | LESS_THAN | GREATER_THAN
  // binop := AND | OR
  // unop :== NOT
  // func := function names
  // var
  
  public void runGrammarBasedParity() {
    Listener.Factory<Event<?, ?, ? extends Double>> listenerFactory = new TabularPrinter<>(Misc.concat(List.of(BASIC_FUNCTIONS, DOUBLE_FUNCTIONS)));
    Random r = new Random(1);
    GrammarBasedProblem<String, List<Tree<Element>>, Double> p;
    
    try {
      p = new SAFEMUV_Grammar_problem(8);
    } catch (IOException e) {
      System.err.printf("Cannot load problem due to %s%n", e);
      return;
    }
    Evolver<Tree<String>, List<Tree<Element>>, Double> evolver = new StandardEvolver<>(
        new it.units.malelab.jgea.problem.booleanfunction.FormulaMapper(),
        new GrammarRampedHalfAndHalf<>(3, 12, p.getGrammar()),
        PartialComparator.from(Double.class).comparing(Individual::getFitness),
        100,
        Map.of(
            new SameRootSubtreeCrossover<>(12), 0.8d,
            new GrammarBasedSubtreeMutation<>(12, p.getGrammar()), 0.2d
        ),
        new Tournament(3),
        new Worst(),
        100,
        true,
        false
    );
       
    try {
      Collection<List<Tree<Element>>> solutions = evolver.solve(
          Misc.cached(p.getFitnessFunction(), 10000),
          new Iterations(100),
          r,
          executorService,
          listenerFactory.build().deferred(executorService)
      );
      System.out.printf("Found %d solutions with %s.%n", solutions.size(), evolver.getClass().getSimpleName());
    } catch (InterruptedException | ExecutionException e) {
      e.printStackTrace();
    }
  }
}
