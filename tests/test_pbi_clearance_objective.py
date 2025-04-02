import unittest
import numpy as np
from ompl import base as ob
from ompl import geometric as og
import pybullet_industrial as pi


class DummyValidityChecker(ob.StateValidityChecker):
    """
    Dummy validity checker that returns a preset clearance value.
    """

    def __init__(self, si, clearance_value: float = 1.0):
        super().__init__(si)
        self.clearance_value = clearance_value

    def isValid(self, state: ob.State) -> bool:
        """
        Always returns True, as this is a dummy validity checker.

        Args:
            state (ob.State): Dummy state (unused).

        Returns:
            bool: Always True.
        """
        return True

    def clearance(self, state: ob.State) -> float:
        """
        Returns the preset clearance value.

        Args:
            state (ob.State): Dummy state (unused).

        Returns:
            float: The preset clearance.
        """
        self.isValid(state)
        return self.clearance_value


class TestPbiClearanceObjective(unittest.TestCase):
    """
    Unit tests for the PbiClearanceObjective class.
    """
    def set_configuration(self, importance=None, target_clearance=None, max_clearance=None):
        """
        Set the configuration parameters for the clearance objective.

        Args:
            importance (float): Importance of the clearance.
            target_clearance (float): Target clearance.
            max_clearance (float): Maximum clearance.
        """
        if importance is not None:
            self.clearance_obj.importance = importance
        if target_clearance is not None:
            self.clearance_obj.target_clearance = target_clearance
        if max_clearance is not None:
            self.clearance_obj.max_clearance = max_clearance

    def setUp(self) -> None:
        """
        Set up a dummy space information and state.
        Default parameters:
            importance = 0.5, target_clearance = 1.0, max_clearance = 2.0.
        """
        # Create bounds and simple setup.
        space = ob.RealVectorStateSpace(1)
        simple_setup = og.SimpleSetup(space)
        self.si = simple_setup.getSpaceInformation()
        # Set the state validity checker to a dummy checker with 0 clearance.
        dummy_checker = DummyValidityChecker(self.si, 0.0)
        simple_setup.setStateValidityChecker(dummy_checker)
        # Create a dummy state.
        self.state = ob.State(space)
        self.state[0] = 0.0



        # Create the clearance objective.
        self.clearance_obj = pi.PbiClearanceObjective(self.si)
        self.set_configuration(0.5, 1.0, 2.0)

    def test_clearance_negative(self):
        """
        Test cost when clearance is negative.
        Expected cost equals importance.
        """
        # Set dummy clearance to a negative value.
        self.si.getStateValidityChecker().clearance_value = -0.1

        # Set the importance to 0.5.
        self.set_configuration(0.5, 1.0, 2.0)
        cost = self.clearance_obj.stateCost(self.state)
        expected = 0.5
        self.assertAlmostEqual(cost.value(), expected, places=5)
        # Set the importance to 0.0.
        self.set_configuration(0.0, 1.0, 2.0)
        cost = self.clearance_obj.stateCost(self.state)
        expected = 0.0
        self.assertAlmostEqual(cost.value(), expected, places=5)
        # Set the importance to 1.0.
        self.set_configuration(1.0, 1.0, 2.0)
        cost = self.clearance_obj.stateCost(self.state)
        expected = 1.0
        self.assertAlmostEqual(cost.value(), expected, places=5)

    def test_clearance_above_max(self):
        """
        Test cost when clearance exceeds max_clearance.
        Expected cost equals 1 - importance.
        """
        # Set dummy clearance above max.
        self.si.getStateValidityChecker().clearance_value = 2.5
        # Set the importance to 0.5.
        self.set_configuration(0.5, 1.0, 2.0)
        cost = self.clearance_obj.stateCost(self.state)
        expected = 0.5
        self.assertAlmostEqual(cost.value(), expected, places=5)
        # Set the importance to 0.0.
        self.set_configuration(0.0, 1.0, 2.0)
        cost = self.clearance_obj.stateCost(self.state)
        expected = 1.0
        self.assertAlmostEqual(cost.value(), expected, places=5)
        # Set the importance to 1.0.
        self.set_configuration(1.0, 1.0, 2.0)
        cost = self.clearance_obj.stateCost(self.state)
        expected = 0.0
        self.assertAlmostEqual(cost.value(), expected, places=5)

    def test_clearance_below_target(self):
        """
        Test cost when clearance is between 0 and target_clearance.
        Expected cost is computed via linear formula.
        """
        # Set dummy clearance below target.
        self.si.getStateValidityChecker().clearance_value = 0.5
        # Set the importance to 0.5.
        self.set_configuration(0.5, 1.0, 2.0)
        cost = self.clearance_obj.stateCost(self.state)
        expected = 0.25
        self.assertAlmostEqual(cost.value(), expected, places=5)
        # Set the importance to 0.0.
        self.set_configuration(0.0, 1.0, 2.0)
        cost = self.clearance_obj.stateCost(self.state)
        expected = 0.0
        self.assertAlmostEqual(cost.value(), expected, places=5)
        # Set the importance to 1.0.
        self.set_configuration(1.0, 1.0, 2.0)
        cost = self.clearance_obj.stateCost(self.state)
        expected = 0.5
        self.assertAlmostEqual(cost.value(), expected, places=5)

    def test_clearance_between_target_max(self):
        """
        Test cost when clearance is between target and max_clearance.
        Expected cost is computed via linear scaling.
        """
        # Set dummy clearance between target and max.
        self.si.getStateValidityChecker().clearance_value = 1.5
        # Set the importance to 0.5.
        self.set_configuration(0.5, 1.0, 2.0)
        cost = self.clearance_obj.stateCost(self.state)
        expected = 0.25
        self.assertAlmostEqual(cost.value(), expected, places=5)
        # Set the importance to 0.0.
        self.set_configuration(0.0, 1.0, 2.0)
        cost = self.clearance_obj.stateCost(self.state)
        expected = 0.5
        self.assertAlmostEqual(cost.value(), expected, places=5)
        # Set the importance to 1.0.
        self.set_configuration(1.0, 1.0, 2.0)
        cost = self.clearance_obj.stateCost(self.state)
        expected = 0.0
        self.assertAlmostEqual(cost.value(), expected, places=5)


if __name__ == '__main__':
    unittest.main()
