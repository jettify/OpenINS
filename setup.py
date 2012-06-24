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
            #Extension("openins.trajectory.navtrajectory_opt", ["openins/trajectory/navtrajectory_opt.c"]),
            Extension("openins.trajectory.navtrajectory_cf", ["openins/trajectory/navtrajectory_cf.c"]),
            Extension("openins.trajectory.navtrajectory_opt2", ["openins/trajectory/navtrajectory_opt2.c"]),
        ])

except ImportError:
    print "Setuptools must be installed - see http://pypi.python.org/pypi/setuptools"
