#!/usr/bin/env python
depsOK = True

try:
    import numpy
except ImportError:
    depsOK = False
    print "NumPy should be installed first from suitable binaries."
    print "See http://numpy.scipy.org/"

try:
    import scipy
except ImportError:
    depsOK = False
    print "SciPy should be installed first from suitable binaries."
    print "See http://www.scipy.org/"

try:
    import sympy
except ImportError:
    depsOK = False
    #TODO: change links
    print "SciPy should be installed first from suitable binaries."
    print "See http://www.scipy.org/"

try:
    import matplotlib
except ImportError:
    depsOK = False
    print "Matplotlib should be installed first from suitable binaries."
    print "See http://matplotlib.sf.net/"

try:
    from setuptools import setup, find_packages
    from setuptools.extension import Extension
    from Cython.Distutils import build_ext




    if depsOK:
        setup(
            name = "openins",
            version = "0.001",
            author = "Mykoa Novik",
            license = "MIT",
            url = "http://www.github.com/phen0m/OpenINS",
            #install_requires = ["sympy", "pyparsing"],
            #packages = find_packages(),
            cmdclass = {'build_ext': build_ext},
                ext_modules = [
            #Extension("OpenINS.trajectory.navtrajectory_opt", ["OpenINS/trajectory/navtrajectory_opt.pyx"]),
            Extension("OpenINS.trajectory.navtrajectorycf", ["OpenINS/trajectory/navtrajectorycf.pyx"]),
            Extension("OpenINS.trajectory.navtrajectoryopt2", ["OpenINS/trajectory/navtrajectoryopt2.pyx"]),


            Extension("OpenINS.trajectory.volume", ["OpenINS/trajectory/volume.pyx"]),
            Extension("OpenINS.trajectory.spam", ["OpenINS/trajectory/spam.pyx"]),
        ])

except ImportError:
    print "Setuptools must be installed - see http://pypi.python.org/pypi/setuptools"
