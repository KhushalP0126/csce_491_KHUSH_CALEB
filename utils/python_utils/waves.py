# Copyright 2021 Jason Bakos, Philip Conrad, Charles Daniels
#
# Distributed as part of the University of South Carolina CSCE491 course
# materials. Please do not redistribute without written authorization.

# This file implements a library which can be used to interact with waves
# in the format prescribed for the CSCE491 labs.

# https://github.com/Nic30/pyDigitalWaveTools
from pyDigitalWaveTools.vcd.parser import VcdParser
from pyDigitalWaveTools.vcd.writer import VcdWriter
from pyDigitalWaveTools.vcd.common import VCD_SIG_TYPE
from pyDigitalWaveTools.vcd.value_format import VcdBitsFormatter

from io import StringIO
import datetime

# https://github.com/Nic30/pyDigitalWaveTools/blob/ab0b89c4a6710c24da68de0e991e6183c056e888/tests/vcdWriter_test.py#L20
class MaskedValue():

    def __init__(self, val, vld_mask):
        self.val = val
        self.vld_mask = vld_mask


class Waves:
    """Waves.

    This object represents a collection of zero or more waves, which are
    assumed to be digital signals of not more than 64 bits each in width, which
    are sampled at various points in time.
    """


    def __init__(this):
        """__init__.

        Instantiates a new collection of waves, containing no sample data. Once
        instantiated, use the .loadText() or .loadVCD() methods to populate
        this object with sample data.

        :param this:
        """

        # List of tuples, where each tuple has the format (timestamp, signals),
        # and signals is a dict where keys are signal names, and values are the
        # signal values at the given timestep.
        #
        # This data structure must be kept sorted by timestamp.
        this.data = []

        # Hash table associating signal names with their widths in bits. All
        # signals in the waveform must have a key in this table.
        this.sizes = {}

    def signals(this): # -> list[str]:
        """signals.

        :returns: a list of signal names contained in this collection of Waves.
        :rtype: list[str]
        """

        return this.sizes.keys()

    def samples(this): # -> int:
        """samples.

        :param this:
        :rtype: int
        :returns: the number of samples recorded in this Waves object.
        """

        return len(this.data)

    def mask(this, signal: str): # -> int:
        """mask.

        Generate a bit mask for the specified signal's width. For example,
        a signal with width 4 would return 0xf.

        :param signal: the signal to generate a mask for.
        :type signal: str
        :returns: the generated signal mask.
        :rtype: int
        :raises KeyError: if the signal is not a known signal name for this
        object.
        """

        if signal not in this.sizes.keys():
            raise KeyError("Unknown signal '{}'".format(signal))

        size = this.sizes[signal]
        mask = 0
        for i in range(size):
            mask = (mask << 1) | 1

        return mask

    def indexOfTime(this, time: float) -> int:
        """indexOfTime.

        This function finds the index into the signal data at which the given
        time is current, that is, the index of the dataum which has a timestamp
        less or equal to the given time, and for which the subsequent data item
        has a timestamp strictly greater than the given time.

        :param time: The time to find the index of.
        :type time: float
        :returns: The index at which the requested time is current.
        :rtype: int
        """

        # Perform a binary search to find the appropriate index into this.data.
        index = int(len(this.data)/2)
        step = int(len(this.data)/2)
        while True:
            # end conditions
            if index >= (len(this.data)-1):
                if step > 2:
                    index = len(this.data)-1
                else:
                    return len(this.data)-1

            if index <= 0:
                if step > 2:
                    index = 0
                else:
                    return 0

            if (this.data[index][0] <= time) and (this.data[index+1][0] > time):
                return index

            # time is too small
            if this.data[index][0] < time:
                index = index + step
            else:
                index = index - step

            step = float(step) / 2.0
            if step < 1.0:
                step = 1
            step = int(step)


    def signalAt(this, signal: str, time: float) -> int:
        """signalAt.

        This function retrieves the value of a signal at a specific point in
        time.

        If no signal data is recorded in this object, then the this function
        returns 0.

        :param signal: The name of the signal.
        :type signal: str
        :param time: The time at which the signal is to be sampled.
        :type time: float
        :returns: The signal value at the requested time.
        :rtype: int
        :raises ValueError: if time is negative.
        :raises KeyError: if signal is not a know signal name for this object.
        """

        if signal not in this.sizes.keys():
            raise KeyError("Unknown signal '{}'".format(signal))

        if time < 0:
            raise ValueError("Time cannot be negative, got {}.".format(time))

        if len(this.data) < 1:
            return 0

        return this.mask(signal) & this.data[this.indexOfTime(time)][1][signal]


    def nextEdge(this, signal: str, time: float, posedge: bool=True, negedge: bool=True): #-> tuple[float, bool]:
        """nextEdge.

        This function finds the time at which the next edge occurs starting
        at the specified time.

        If no signal data is recorded in this object, then this function
        returns +Inf, False.

        :param signal: The name of the signal.
        :type signal: str
        :param time: The time of the earliest edge to be reported.
        :type time: float
        :param posedge: If this parameter is True, then rising edges will be
            reported, otherwise they will be omitted.
        :type posedge: bool
        :param negedge: If this parameter is True, then falling edges will be
        reported, otherwise they will be omitted.
        :type negedge: bool
        :returns: The first return value is the time at which the next edge
            occurs, or +Inf if none was found. The second return value is True
            if an edge was found, and False otherwise.
        :rtype: tuple[float, bool]
        :raises ValueError: if time is negative.
        :raises KeyError: if signal is not a know signal name for this object.
        """

        if signal not in this.sizes.keys():
            raise KeyError("Unknown signal '{}'".format(signal))

        if time < 0:
            raise ValueError("Time cannot be negative, got {}.".format(time))

        if len(this.data) < 1:
            return float('inf'), False

        index = this.indexOfTime(time) 

        # definitionally, an edge cannot occur at time 0, and it messes it up
        # when we check for the delta
        if index == 0:
            index = 1

        while True:
            if index >= (len(this.data)-1):
                return float('inf'), False

            if this.data[index-1][0] < time:
                index += 1
                continue

            if posedge and (this.data[index-1][1][signal] < this.data[index][1][signal]):
                return this.data[index][0], True

            if negedge and (this.data[index-1][1][signal] > this.data[index][1][signal]):
                return this.data[index][0], True

            index += 1

    def toText(this) -> str:
        """toText.

        This function returns the contents of the wave object in the text
        format used in this course.

        :rtype: str
        """

        signals = list(this.sizes.keys())

        s = "{}\n".format(len(this.data))
        s += "{}\n".format("\t".join(signals))
        s += "{}".format("\t".join([str(this.sizes[k]) for k in signals]))
        for index in range(len(this.data)):
            s += "\n{}\t{}".format(str(this.data[index][0]), "\t".join([str(this.data[index][1][k]) for k in signals]))


        return s

    def loadText(this, text: str):
        """loadText.

        This function loads a file stored in the text format used in this
        course. Any data already stored in this object is destroyed.

        :param text: The contents of the text file to load.
        :type text: str
        :raises ValueError: If a syntax error occurs while parsing the text. If
            an exception occurs while parsing, the state of the object being
            parsed into is undefined.
        """

        linum = 1
        trueline = 1  # lines including comments and empties, just for error messages
        signals = []
        widths = []
        for line in [l.strip() for l in text.split("\n")]:

            # ignore comments
            if (len(line.strip()) >= 1) and (line.strip()[0] == '#'):
                trueline += 1
                continue

            # ignore empty lines
            if len(line.strip()) == 0:
                trueline += 1
                continue

            if linum == 1:
                # this is just the number of records, we don't need to know
                # this
                pass

            elif linum == 2:
                # parse the list of signals
                for s in line.split("\t"):
                    signals.append(s.strip())

            elif linum == 3:
                # parse the signal widths
                i = 0
                for w in line.split("\t"):
                    wv = 0
                    w = w.strip()
                    try:
                        wv = int(w)
                    except Exception as e:
                        if i < len(signals):
                            raise ValueError("Could not parse signal width '{}' for signal '{}' due to error: '{}'".format(w, signals[i], e))
                        else:
                            raise ValueError("Could not parse signal width '{}' for out of bounds signal due to error: '{}'".format(w, e))

                    widths.append(wv)

                if len(widths) != len(signals):
                    raise ValueError("Number of signals ({}) must match number of signal widths ({})".format(len(signals), len(widths)))

                this.sizes = {}
                for i in range(len(signals)):
                    this.sizes[signals[i]] = widths[i]

                this.data = []


            else:

                line = [f.strip() for f in line.split("\t")]

                if len(line) != 1 + len(signals):
                    raise ValueError("On line {}, line must contain {} components, but has {}".format(trueline, 1 + len(signals), len(line)))

                timestamp = line[0]

                try:
                    timestamp = float(timestamp)

                except Exception as e:
                    raise ValueError("On line {}, failed to parse timestamp '{}' due to error: '{}'".format(trueline, line[0], e))

                if timestamp < 0:
                    raise ValueError("On line {}, timestamp {} is negative, which is not permitted".format(trueline, timestamp))

                if linum > 4:  # we have at least one previous data point
                    if timestamp <= this.data[-1][0]:
                        raise ValueError("On line {}, timestamp {} moves backwards - timestamps must be monotonically increasing".format(trueline, timestamp))

                values = {}
                for i in range(len(signals)):
                    n = line[i+1]
                    try:
                         n = int(n)
                    except Exception as e:
                        raise ValueError("On line {}, failed to parse signal value for signal '{}' due to error: '{}'".format(trueline, signals[i], e))

                    values[signals[i]] = n

                this.data.append( (timestamp, values) )


            linum += 1
            trueline += 1


    def __enumerateVCDSignals(this, scope):
        """__enumerateVCDSignals.

        Given a pyDigitalaveTools scope, enumerate all of the signals in it to
        a flat dictionary.

        :param scope:
        """

        res = {}

        name = scope.name + "."
        if name == "root.":
            name = ""

        for child in scope.children.values():
            if "children" in child.__dict__:
                # nested scope
                nested = this.__enumerateVCDSignals(child)
                for signal in nested:
                    res[name + signal] = nested[signal]
            else:
                #child
                res[name + child.name] = child

        return res


    def loadVCD(this, text: str, timescale: float=0.0001):
        """loadVCD.

        This method overwrites whatever data is stored in this Waves object
        with the contents of a VCD file.

        This method does not support arbitrary VCD files. Scopes will be
        flattened, and any don't-care or high-impedance values will be
        converted to 0. Values of types other than 'wire' are also ignored
        entirely.

        :param text: VCD file contents to parse.
        :type text: str
        :param timescale: VCD timestamps are multiplied by this value.
        :type timescale: float
        """

        this.data = []
        this.sizes = {}

        # parse the VCD file
        vcd = VcdParser()
        vcd.parse_str(text)
        sigs = this.__enumerateVCDSignals(vcd.scope)

        # extract all names and widths
        for k in sigs:
            sig = sigs[k]
            if sig.sigType != "wire":
                continue

            this.sizes[k] = sig.width


        #  Walk forward across all signals to get their values at a given
        #  time. The timestamp is maintained in the VCD context.
        timestamp = 0
        while True:
            if (timestamp-1) > vcd.now:
                break

            change = False
            newVals = {}
            for k in sigs:
                sig = sigs[k]
                if sig.sigType != "wire":
                    continue

                if len(sig.data) < 1:
                    continue

                i = 1
                while sig.data[i][0] < timestamp:
                    i += 1
                    if i >= len(sig.data):
                        break
                i -= 1

                valueAtTimestamp = sig.data[i][1]
                valueAtTimestamp = valueAtTimestamp.replace("X", "0")
                valueAtTimestamp = valueAtTimestamp.replace("x", "0")
                valueAtTimestamp = valueAtTimestamp.replace("Z", "0")
                valueAtTimestamp = valueAtTimestamp.replace("z", "0")

                if valueAtTimestamp[0] == 'b':
                    valueAtTimestamp = int(valueAtTimestamp[1:], 2)

                else:
                    valueAtTimestamp = int(valueAtTimestamp)

                newVals[k] = valueAtTimestamp & this.mask(k)

                if (len(this.data) == 0) or (valueAtTimestamp != this.data[-1][1][k]):
                    change = True

            if change:
                this.data.append((timestamp * timescale, newVals))

            timestamp += 1

    def toVCD(this, timescale: float=10000):
        """toVCD.

        This method returns a string representing a VCD file containing the
        data stored in this waves object.

        :param this:
        :param timescale: time values will be multiplied by this amount
        """

        f = StringIO("")
        w = VcdWriter(oFile=f)

        w.date(datetime.datetime.now())
        w.timescale(1)

        with w.varScope("root") as m:
            for s in this.signals():
                m.addVar(s, s, VCD_SIG_TYPE.WIRE, this.sizes[s], VcdBitsFormatter())

        w.enddefinitions()

        for i in range(len(this.data)):
            t, vals = this.data[i]
            t *= timescale
            for s in this.signals():
                w.logChange(t, s, MaskedValue(vals[s], this.mask(s)), None)

        f.seek(0)
        res = f.read()
        f.close()
        return res




