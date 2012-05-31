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
    import matplotlib
except ImportError:
    depsOK = False
    print "Matplotlib should be installed first from suitable binaries."
    print "See http://matplotlib.sf.net/"

try:
    from setuptools import setup, find_packages
    from setuptools.extension import Extension
    if depsOK:
        setup(
            name = "openins",
            version = "0.0",
            author = "Mykoa Novik",
            license = "MIT",
            url = "http://www.github.com/phen0m/OpenINS",
            install_requires = ["simpy", "pyparsing"],
            packages = find_packages(),
            include_dirs = [numpy.get_include()],
            ext_modules = [
                Extension("OpenINS.maths.quaternions",
                    ['OpenINSS/maths/quaternions.c']),
                Extension("OpenINS.maths.quat_splines",
                    ['OpenINS/maths/quat_splines.c']),
                Extension("OpenINS.maths.vectors",['OpenINS/maths/vectors.c']),
                Extension("OpenINS.maths.natural_neighbour",[
                    'OpenINS/maths/natural_neighbour/utils.c',
                    'OpenINS/maths/natural_neighbour/delaunay.c',
                    'OpenINS/maths/natural_neighbour/natural.c',
                    'OpenINS/maths/natural_neighbour.c'])]
        )
except ImportError:
    print "Setuptools must be installed - see http://pypi.python.org/pypi/setuptools"
