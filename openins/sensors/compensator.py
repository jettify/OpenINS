#"""
#Base module for sensor's compensation
#algorithms
#"""
#
#import numpy as np
#
#from base import DataHandler
#
#
#class SensorCompensator(DataHandler):
#    """
#    Implements compensation algorithm for
#    static errors
#    """
#    def __init__(self):
#
#        self._param = 0
#
#    @property
#    def parameter(self):
#        """
#        Returns parameter value, bias or scale factor
#        for instance.
#        """
#        return self._param
#
#    @parameter.setter
#    def _set_pamam(self, value):
#        """
#        Parameter setter
#        """
#
#        self._param = value
