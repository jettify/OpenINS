# Makefile for OpenINS documentation and tests
#

.PHONY: default clean clean_c all c test build
APP=openins
PWD=$(shell pwd)


#export PYTHONPATH:=$(PWD):${PYTHONPATH}


default:
	@echo "This Makefile is only for building documentation and running tests."
	@echo "To build and install OpenINS, run:"
	@echo
	@echo "    python setup.py build"
	@echo "    python setup.py install (as root or administrator)"
	@echo
	@echo "Or, from the parent directory, with setuptools installed:"
	@echo
	@echo "    easy_install openins"
	@echo
	@echo "This will also download and install required dependencies."


c:
	@echo "Generate *.c files from *.pyx:"
	cython -v `find . -name \*.pyx`
	#python setup.py build_ext --inplace

build_c:
	@echo "Compiling *.c extentions"
	python setup.py build_ext --inplace

sphinx:
	cd docs; sphinx-build -b html . build/html/

test:
	@echo "Run all tests:"
	python -m unittest discover -v -p 'test_*.py'

clean:
	@echo "Clean project from temp files:"
	find . -type f -name "*.pyc" -exec rm -r -v {} \;
	find . -type f -name "*.o" -exec rm -r -v {} \;
	find . -type f -name "*.so" -exec rm -r -v {} \;
	@rm -rf -v build

clean_c:
	@echo "Delete *.c files:"
	find . -type f -name "*.c" -exec rm -r -v {} \;

