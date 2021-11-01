#!/bin/sh

DATESTR=$(date '+%Y_%m_%d_%H_%M');
OUTDIR=temp-res/jmetal/$DATESTR;
mkdir $OUTDIR
echo "Saving to $OUTDIR"
cp /tmp/*.csv $OUTDIR
cp tempLog-*.res $OUTDIR
cp jmetal-final*.res $OUTDIR
cp populationAtEval*.res $OUTDIR
cp mutation.log $OUTDIR
cp crossover.log $OUTDIR
cp jMetal.log $OUTDIR
