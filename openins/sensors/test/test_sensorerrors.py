#import unittest
#import random
#import numpy as np
#
#from sensors.sensorerrors import Bias
#from sensors.sensorerrors import ScaleFactor
#from sensors.base import Sensor
#
#class test_StaticError(unittest.TestCase):
#
#    def setUp(self):
#
#        # set error parameters
#        self.bias  = random.random()
#        self.scalef = random.random()
#
#
#        self.b = Bias(self.bias)
#        self.sf = ScaleFactor(self.scalef)
#
#        # ADXL345 ideal sensor
#        self.ADXL345_i = Sensor()
#
#        # ADXL345 sensor corrupted by bias
#        self.ADXL345_b = Sensor()
#        self.ADXL345_b.add_handler(self.b)
#        # ADXL345 sensor corrupter by scale factor
#        self.ADXL345_sf = Sensor()
#        self.ADXL345_sf.add_handler(self.sf)
#        # ADXL345 sensor corrupter by bias and scale factor
#        self.ADXL345_bsf = Sensor()
#        self.ADXL345_bsf.add_handler(self.b)
#        self.ADXL345_bsf.add_handler(self.sf)
#        # generate input (ideal sensor measurements)
#        self.input_data = np.arange(-10.,10.,1.)
#
#    def tearDown(self):
#
#        del self.input_data
#    def test_ideal(self):
#        output = self.ADXL345_i.run(self.input_data)
#        zarray = output - self.input_data
#        [self.assertEqual(data,0) for data in zarray]
#
#
#    def test_bias(self):
#
#        output = self.ADXL345_b.run(self.input_data)
#        zarray = output - self.input_data - self.bias
#        [self.assertAlmostEqual(data,0 , places=12) for data in zarray]
#
#    def test_sf(self):
#
#        output = self.ADXL345_sf.run(self.input_data)
#        zarray = output/self.scalef - self.input_data
#        [self.assertAlmostEqual(data,0 , places=12) for data in zarray]
#
#    def test_sfbias(self):
#
#        output = self.ADXL345_bsf.run(self.input_data)
#        zarray = (output)/self.scalef - self.input_data  - self.bias
#        [self.assertAlmostEqual(data,0 , places=12) for data in zarray]
#
#
#if __name__ == "__main__":
#    pass
#    #unittest.main()
