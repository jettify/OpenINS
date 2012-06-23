import inspect

def get_implementations(module, superclass):
    """
    Get algorithm implementations from a module.

    Returns
    -------
    All concrete subclasses of the superclass found in the module.
    """
    predicate = lambda obj: inspect.isclass(obj) \
            and not inspect.isabstract(obj)
    return( imp for name, imp in inspect.getmembers(module,
            predicate) if issubclass(imp, superclass) )
