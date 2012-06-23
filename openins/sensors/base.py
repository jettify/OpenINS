from abc import ABCMeta, abstractproperty, abstractmethod


class DataHandler(object):
    """
    Base class for data handler. Filters, functions witch add static or
    stochastic errors.
    """

    __metaclass__ = ABCMeta

    @abstractmethod
    def handle(self, tk, data):
        """
        Basic handle function.
        """
        return tk, data


class Sensor(object):
    """
    Class witch combines StaticError, and exec functions one by one.
    """
    def __init__(self):

        self.errors_gen = []
        self.filters = []
        self.compensator = []
        # function just do nothing, returns input.
        # looks like ideal sensor error

    def add_handler(self, handler):
        """
        Save all StaticErrors objects in list.

        Parameters
        ----------
        handler: DataHandler based object
        """
        if type(handler) == SensorError:
            self.errors.append(error_handler)

        elif type(handler) == SensorFilter:
            self.filters.append(filter_handler)

        else:
            raise TypeError('command must be instance of SensorError')

    def run(self,tk,data):
        """
        Add to input data errors and then filter it

        Parameters
        ----------
        tk: float, array like
            seconds, time sequence with sample time dt
        data: float, array like
            input data
        """

        if not len(tk) == len(data):
            raise ValueError('tk and data must have same size')

        handle_sequence = self.errors + self.filters
        if not handle_sequence:
            handle_sequence.append('')
        for c in handle_sequence:
            #data = c.handle(data)
            #data = f.
            vfunc = np.vectorize(c.handle)
            data = np.array(vfunc(tk,data))

        return tk, data
