# Makefile for OpenINS documentation and tests
#

.PHONY: default clean all
APP=OpenINS
PWD=$(shell pwd)/$(APP)


export PYTHONPATH:=$(PWD):${PYTHONPATH}


default:
	@echo "This Makefile is only for building documentation and running tests."
	@echo "To build and install OpenINS, run:"

sphinx:
	cd docs; sphinx-build -b html . build/html/

test:
	python -m unittest discover -v -p 'test_*.py'

clean:
	@echo "Clean project from *.pyc files"
	find . -type f -name "*.pyc" -exec rm -r -v {} \;


