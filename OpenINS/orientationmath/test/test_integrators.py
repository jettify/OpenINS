"""
Tests for integration methods.
"""


from __future__ import division
import numpy as np
import unittest

from OpenINS.orientationmath.integrators import RectangleRule, TrapeziumRule

x = np.arange(0., 5, 0.0005)
dt = x[1]-x[0]

functions = ( lambda x: x,
              lambda x: x**2,
              lambda x: np.cos(x),
              lambda x: np.e**x )

integrals = ( lambda x: 1/2 * x**2,
              lambda x: 1/3 * x**3,
              lambda x: np.sin(x),
              lambda x: np.e**x)



methods = (RectangleRule, TrapeziumRule)
tols = (1, 4)

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
            for function, integral in zip (functions, integrals):
                test_int, ref_int = check_integral(method, function, integral)
                for test, ref in zip (test_int, ref_int):
                    self.assertAlmostEqual(test, ref, places=tol)



#class IntegratorVectorTest(unittest.TestCase):
#    """
#    Test integrators with vector function.
#    """
#    def foo(self, x):
#        return np.array([f(x) for f in functions])
#
#    def int_foo(self, x):
#        return np.array([i(x) for i in integrals])
#
#    def test_integral(self):
#        """
#        Try to integrate each function and test result with exact
#        solution.
#        """
#        for  method, tol in zip (methods, tols):
#            test_int, ref_int = check_integral(method, self.foo, self.int_foo)
#            for test, ref in zip (test_int, ref_int):
#                self.assertAlmostEqual(test, ref, places=tol)