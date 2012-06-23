"""
Tests for integration methods.
"""


from __future__ import division
import numpy as np
import unittest
import orientationmath.integrators
from testing.inspection import get_implementations
from orientationmath.integrators import SimpsonRule
from orientationmath.integrators import TrapeziumRule

from visualisation.plotter import plot_basic

x = np.arange(0., 5, 0.0005)
dt = x[1]-x[0]

functions = ( lambda x: x,
              lambda x: x**2,
              lambda x: np.cos(x),
              lambda x: np.e**x )

integrals = ( lambda x: 1/2. * x**2,
              lambda x: 1/3. * x**3,
              lambda x: np.sin(x),
              lambda x: np.e**x)


methods  = get_implementations(orientationmath.integrators,
                               orientationmath.integrators.Integrator)
tols = (0, 4, 4)

def check_integral(method, function, integral):
    """
    Find integral of function with given method.
    """
    estimated_integral = np.empty_like(x)
    integrator = method(integral(0.))
    for i, s in enumerate(x[1:]):
        estimated_integral[i+1] = integrator(function(s), dt=dt)

    return estimated_integral[1:], integral(x[1:])



class IntegratorTest(unittest.TestCase):
    """
    Test integrators on known functions.
    """

    def test_integral(self):
        """
        Try to integrate each function and test result with exact
        solution.
        """
        for  method, tol in zip (methods, tols):
            print method
            for function, integral in zip (functions, integrals):
                print integral
                test_int, ref_int = check_integral(method, function, integral)
                for test, ref in zip (test_int, ref_int):
                    self.assertAlmostEqual(test, ref, places=tol)



class CheckSimpson(unittest.TestCase):
    """

    """
    def test_integral(self):

        func = np.cos
        integ = np.sin

        init_v = integ(0.)

        integrator = TrapeziumRule(init_v)
#        integrator = SimpsonRule(init_v)


        x = np.arange(0., 5, 0.01)
        dt = x[1]-x[0]
        estimated_integral = np.empty_like(x)
        real_integral = integ(x)

        estimated_integral[0] = init_v
        for i, s in enumerate(x[1:]):
            estimated_integral[i+1] = integrator(func(s), dt=dt)

        plot_basic(x, estimated_integral-real_integral)
