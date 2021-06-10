#!/usr/bin/python3
import pyecore
from pyecore.resources import ResourceSet, URI

rset = ResourceSet()
mm_resource = rset.get_resource(URI('/home/jharbin/academic/atlas/atlas-middleware/ecore-metamodels/com.github.atlas.dsl/fuzzspec.ecore'))
mm_root = mm_resource.contents[0]
rset.metamodel_registry[mm_root.nsURI] = mm_root

m = rset.get_resource(URI('/home/jharbin/academic/atlas/atlas-middleware/atlas-models/case-study-ros/fuzzing-example.model'))
model_root = m.contents[0]

def print_all_contents(element):
    contents = element.eContents
    for aname in dir(element):
        print(aname + "=" + str(element.eGet(aname)))
    for e in contents:
        print_all_contents(e)


print_all_contents(model_root)

def new_model():
    model_root = FuzzingSpecification();
