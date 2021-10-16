package fuzzexperiment.runner.jmetal.grammar;

import it.units.malelab.jgea.core.operator.Mutation;
import it.units.malelab.jgea.representation.tree.Tree;

import java.util.Collections;
import java.util.List;
import java.util.Random;

public class GrammarBasedSubtreeMutation<T> implements Mutation<Tree<T>> {

  private final int maxDepth;
  private GrowGrammarTreeFactory<T> factory;

  public GrammarBasedSubtreeMutation(int maxDepth, Grammar<T> grammar) {
    this.maxDepth = maxDepth;
    factory = new GrowGrammarTreeFactory<>(0, grammar);
  }

  @Override
  public Tree<T> mutate(Tree<T> parent, Random random) {
    Tree<T> child = Tree.copyOf(parent);
    List<Tree<T>> nonTerminalTrees = child.topSubtrees();
    Collections.shuffle(nonTerminalTrees, random);
    boolean done = false;
    for (Tree<T> toReplaceSubTree : nonTerminalTrees) {
      Tree<T> newSubTree = factory.build(random, toReplaceSubTree.content(), toReplaceSubTree.height());
      if (newSubTree != null) {
        toReplaceSubTree.clearChildren();
        newSubTree.childStream().forEach(toReplaceSubTree::addChild);
        done = true;
        break;
      }
    }
    if (!done) {
      return null;
    }
    return child;
  }

}