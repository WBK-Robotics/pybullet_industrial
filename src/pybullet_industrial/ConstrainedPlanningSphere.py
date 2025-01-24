from __future__ import print_function
from ompl import util as ou
from ompl import base as ob
from ompl import geometric as og
from ompl import tools as ot

import sys
import datetime
import argparse
import math
import numpy as np


def addSpaceOption(parser):
    parser.add_argument("-s", "--space", default="PJ",
                        choices=["PJ", "AT", "TB"],
                        help="""Choose which constraint handling methodology to use. One of:
        PJ - Projection (Default)
        AT - Atlas
        TB - Tangent Bundle.""")


def addPlannerOption(parser):
    parser.add_argument("-p", "--planner", default="RRT",
                        help="Comma-separated list of which motion planner to use (multiple if "
                        "benchmarking, one if planning).\n Choose from, e.g.:\n"
                        "RRT (Default), RRTConnect, RRTstar, "
                        "EST, BiEST, ProjEST, "
                        "BITstar, "
                        "PRM, SPARS, "
                        "KPIECE1, BKPIECE1.")


def addConstrainedOptions(parser):
    group = parser.add_argument_group("Constrained planning options")
    group.add_argument("-d", "--delta", type=float, default=ob.CONSTRAINED_STATE_SPACE_DELTA,
                       help="Step-size for discrete geodesic on manifold.")
    group.add_argument("--lambda", type=float, dest="lambda_", metavar="LAMBDA",
                       default=ob.CONSTRAINED_STATE_SPACE_LAMBDA,
                       help="Maximum `wandering` allowed during atlas traversal. Must be greater "
                       "than 1.")
    group.add_argument("--tolerance", type=float, default=ob.CONSTRAINT_PROJECTION_TOLERANCE,
                       help="Constraint satisfaction tolerance.")
    group.add_argument("--time", type=float, default=5.,
                       help="Planning time allowed.")
    group.add_argument("--tries", type=int, default=ob.CONSTRAINT_PROJECTION_MAX_ITERATIONS,
                       help="Maximum number sample tries per sample.")
    group.add_argument("-r", "--range", type=float, default=0.,
                       help="Planner `range` value for planners that support this parameter. "
                       "Automatically determined otherwise (when 0).")

def list2vec(l):
    ret = ou.vectorDouble()
    for e in l:
        ret.append(e)
    return ret

def clearSpaceAndPlanner(planner):
    planner.getSpaceInformation().getStateSpace().clear()
    planner.clear()


def addAtlasOptions(parser):
    group = parser.add_argument_group("Atlas options")
    group.add_argument("--epsilon", type=float, default=ob.ATLAS_STATE_SPACE_EPSILON,
                       help="Maximum distance from an atlas chart to the manifold. Must be "
                       "positive.")
    group.add_argument("--rho", type=float, default=ob.CONSTRAINED_STATE_SPACE_DELTA *
                       ob.ATLAS_STATE_SPACE_RHO_MULTIPLIER,
                       help="Maximum radius for an atlas chart. Must be positive.")
    group.add_argument("--exploration", type=float, default=ob.ATLAS_STATE_SPACE_EXPLORATION,
                       help="Value in [0, 1] which tunes balance of refinement and exploration in "
                       "atlas sampling.")
    group.add_argument("--alpha", type=float, default=ob.ATLAS_STATE_SPACE_ALPHA,
                       help="Maximum angle between an atlas chart and the manifold. Must be in "
                       "[0, PI/2].")
    group.add_argument("--bias", action="store_true",
                       help="Sets whether the atlas should use frontier-biased chart sampling "
                       "rather than uniform.")
    group.add_argument("--no-separate", action="store_true",
                       help="Sets that the atlas should not compute chart separating halfspaces.")
    group.add_argument("--charts", type=int, default=ob.ATLAS_STATE_SPACE_MAX_CHARTS_PER_EXTENSION,
                       help="Maximum number of atlas charts that can be generated during one "
                       "manifold traversal.")


class ConstrainedProblem(object):

    def __init__(self, spaceType, space, constraint, options):
        self.spaceType = spaceType
        self.space = space
        self.constraint = constraint
        self.constraint.setTolerance(options.tolerance)
        self.constraint.setMaxIterations(options.tries)
        self.options = options
        self.bench = None
        self.request = None
        self.pp = None

        if spaceType == "PJ":
            ou.OMPL_INFORM("Using Projection-Based State Space!")
            self.css = ob.ProjectedStateSpace(space, constraint)
            self.csi = ob.ConstrainedSpaceInformation(self.css)
        elif spaceType == "AT":
            ou.OMPL_INFORM("Using Atlas-Based State Space!")
            self.css = ob.AtlasStateSpace(space, constraint)
            self.csi = ob.ConstrainedSpaceInformation(self.css)
        elif spaceType == "TB":
            ou.OMPL_INFORM("Using Tangent Bundle-Based State Space!")
            self.css = ob.TangentBundleStateSpace(space, constraint)
            self.csi = ob.TangentBundleSpaceInformation(self.css)

        self.css.setup()
        self.css.setDelta(options.delta)
        self.css.setLambda(options.lambda_)
        if not spaceType == "PJ":
            self.css.setExploration(options.exploration)
            self.css.setEpsilon(options.epsilon)
            self.css.setRho(options.rho)
            self.css.setAlpha(options.alpha)
            self.css.setMaxChartsPerExtension(options.charts)
            if options.bias:
                self.css.setBiasFunction(ob.AtlasChartBiasFunction(lambda c, atlas=self.css:
                                         atlas.getChartCount() - c.getNeighborCount() + 1.))
            if spaceType == "AT":
                self.css.setSeparated(not options.no_separate)
            self.css.setup()
        self.ss = og.SimpleSetup(self.csi)

    def setStartAndGoalStates(self, start, goal):
        # Create start and goal states
        if self.spaceType == "AT" or self.spaceType == "TB":
            self.css.anchorChart(start())
            self.css.anchorChart(goal())

        # Setup problem
        self.ss.setStartAndGoalStates(start, goal)

    def getPlanner(self, plannerName, projectionName=None):
        planner = eval('og.%s(self.csi)' % plannerName)
        try:
            if self.options.range == 0:
                if not self.spaceType == "PJ":
                    planner.setRange(self.css.getRho_s())
            else:
                planner.setRange(self.options.range)
        except:
            pass
        try:
            if projectionName:
                planner.setProjectionEvaluator(projectionName)
        except:
            pass
        return planner

    def setPlanner(self, plannerName, projectionName=None):
        self.pp = self.getPlanner(plannerName, projectionName)
        self.ss.setPlanner(self.pp)

    def solveOnce(self, output=False, name="ompl"):
        self.ss.setup()
        stat = self.ss.solve(self.options.time)

        if stat:
            # Get solution and validate
            path = self.ss.getSolutionPath()
            if not path.check():
                ou.OMPL_WARN("Path fails check!")

            if stat == ob.PlannerStatus.APPROXIMATE_SOLUTION:
                ou.OMPL_WARN("Solution is approximate.")

            # Simplify solution and validate simplified solution path.
            ou.OMPL_INFORM("Simplifying solution...")
            self.ss.simplifySolution(5.)

            simplePath = self.ss.getSolutionPath()
            ou.OMPL_INFORM("Simplified Path Length: %.3f -> %.3f" %
                           (path.length(), simplePath.length()))

            if not simplePath.check():
                ou.OMPL_WARN("Simplified path fails check!")

            # Interpolate and validate interpolated solution path.
            ou.OMPL_INFORM("Interpolating path...")
            path.interpolate()

            if not path.check():
                ou.OMPL_WARN("Interpolated simplified path fails check!")

            ou.OMPL_INFORM("Interpolating simplified path...")
            simplePath.interpolate()

            if not simplePath.check():
                ou.OMPL_WARN("Interpolated simplified path fails check!")

            if output:
                ou.OMPL_INFORM("Dumping path to `%s_path.txt`." % name)
                with open('%s_path.txt' % name, 'w') as pathfile:
                    print(path.printAsMatrix(), file=pathfile)

                ou.OMPL_INFORM(
                    "Dumping simplified path to `%s_simplepath.txt`." % name)
                with open("%s_simplepath.txt" % name, 'w') as simplepathfile:
                    print(simplePath.printAsMatrix(), file=simplepathfile)
        else:
            ou.OMPL_WARN("No solution found.")

        return stat

    def setupBenchmark(self, planners, problem):
        if not ot:
            print("Benchmarking not available, no ompl.tools")
            sys.exit(0)

        self.bench = ot.Benchmark(self.ss, problem)

        self.bench.addExperimentParameter(
            "n", "INTEGER", str(self.constraint.getAmbientDimension()))
        self.bench.addExperimentParameter(
            "k", "INTEGER", str(self.constraint.getManifoldDimension()))
        self.bench.addExperimentParameter(
            "n - k", "INTEGER", str(self.constraint.getCoDimension()))
        self.bench.addExperimentParameter("space", "INTEGER", self.spaceType)

        self.request = ot.Benchmark.Request()
        self.request.maxTime = self.options.time
        self.request.maxMem = 1e9
        self.request.runCount = 100
        self.request.timeBetweenUpdates = 0.1
        self.request.saveConsoleOutput = False
        for planner in planners:
            self.bench.addPlanner(self.getPlanner(planner, problem))

        self.bench.setPreRunEvent(ot.PreSetupEvent(clearSpaceAndPlanner))

    def runBenchmark(self):
        self.bench.benchmark(self.request)
        filename = str(datetime.datetime.now()) + '_' + \
            self.bench.getExperimentName() + '_' + self.spaceType
        self.bench.saveResultsToFile(filename)

    def atlasStats(self):
        # For atlas types, output information about size of atlas and amount of
        # space explored
        if self.spaceType == "AT" or self.spaceType == "TB":
            ou.OMPL_INFORM("Atlas has %d charts" % self.css.getChartCount())
            if self.spaceType == "AT":
                ou.OMPL_INFORM("Atlas is approximately %.3f%% open" %
                               self.css.estimateFrontierPercent())

    def dumpGraph(self, name):
        ou.OMPL_INFORM("Dumping planner graph to `%s_graph.graphml`." % name)
        data = ob.PlannerData(self.csi)
        self.pp.getPlannerData(data)

        with open("%s_graph.graphml" % name, "w") as graphfile:
            print(data.printGraphML(), file=graphfile)

        if self.spaceType == "AT" or self.spaceType == "TB":
            ou.OMPL_INFORM("Dumping atlas to `%s_atlas.ply`." % name)
            with open("%s_atlas.ply" % name, "w") as atlasfile:
                print(self.css.printPLY(), file=atlasfile)





class SphereConstraint(ob.Constraint):

    def __init__(self):
        super(SphereConstraint, self).__init__(3, 1)

    def function(self, x, out):
        out[0] = np.linalg.norm(x) - 1

    def jacobian(self, x, out):
        nrm = np.linalg.norm(x)
        if np.isfinite(nrm) and nrm > 0:
            out[0, :] = x / nrm
        else:
            out[0, :] = [1, 0, 0]


class SphereProjection(ob.ProjectionEvaluator):

    def __init__(self, space):
        super(SphereProjection, self).__init__(space)

    def getDimension(self):
        return 2

    def defaultCellSizes(self):
        self.cellSizes_ = list2vec([.1, .1])

    def project(self, state, projection):
        projection[0] = math.atan2(state[1], state[0])
        projection[1] = math.acos(state[2])


def obstacles(x):
    if x[2] > -0.8 and x[2] < -0.6:
        if x[1] > -0.05 and x[1] < 0.05:
            return x[0] > 0
        return False
    elif x[2] > -0.1 and x[2] < 0.1:
        if x[0] > -0.05 and x[0] < 0.05:
            return x[1] < 0
        return False
    elif x[2] > 0.6 and x[2] < 0.8:
        if x[1] > -0.05 and x[1] < 0.05:
            return x[0] < 0
        return False
    return True


def spherePlanningOnce(cp, plannername, output):
    cp.setPlanner(plannername, "sphere")

    # Solve the problem
    stat = cp.solveOnce(output, "sphere")

    if output:
        ou.OMPL_INFORM("Dumping problem information to `sphere_info.txt`.")
        with open("sphere_info.txt", "w") as infofile:
            print(cp.spaceType, file=infofile)

    cp.atlasStats()
    if output:
        cp.dumpGraph("sphere")
    return stat


def spherePlanningBench(cp, planners):
    cp.setupBenchmark(planners, "sphere")
    cp.runBenchmark()


def spherePlanning(options):
    # Create the ambient space state space for the problem.
    rvss = ob.RealVectorStateSpace(3)
    bounds = ob.RealVectorBounds(3)
    bounds.setLow(-2)
    bounds.setHigh(2)
    rvss.setBounds(bounds)

    # Create our constraint.
    constraint = SphereConstraint()

    cp = ConstrainedProblem(options.space, rvss, constraint, options)
    cp.css.registerProjection("sphere", SphereProjection(cp.css))

    start = ob.State(cp.css)
    goal = ob.State(cp.css)
    start[0] = 0
    start[1] = 0
    start[2] = -1
    goal[0] = 0
    goal[1] = 0
    goal[2] = 1
    cp.setStartAndGoalStates(start, goal)
    cp.ss.setStateValidityChecker(ob.StateValidityCheckerFn(obstacles))

    planners = options.planner.split(",")
    if not options.bench:
        spherePlanningOnce(cp, planners[0], options.output)
    else:
        spherePlanningBench(cp, planners)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-o", "--output", action="store_true",
                        help="Dump found solution path (if one exists) in plain text and planning "
                        "graph in GraphML to `sphere_path.txt` and `sphere_graph.graphml` "
                        "respectively.")
    parser.add_argument("--bench", action="store_true",
                        help="Do benchmarking on provided planner list.")
    addSpaceOption(parser)
    addPlannerOption(parser)
    addConstrainedOptions(parser)
    addAtlasOptions(parser)

    spherePlanning(parser.parse_args())

