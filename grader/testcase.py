# Copyright 2021 Jason Bakos, Philip Conrad, Charles Daniels
#
# Distributed as part of the University of South Carolina CSCE491 course
# materials. Please do not redistribute without written authorization.

# This file implements routines for loading and executing test cases on
# student code under test.

import subprocess
import os
import sys
import pathlib
import shutil
import difflib

from . import g
from . import builder

class TestCase:
    def __init__(this, p):
        """__init__.

        This function loads a test case directory to create a TestCase object.


        A testcase directory has the following files:

        * description.txt - (optional) a textual description of the test case
        * input.txt - the input file for the test case
        * output.txt - the expected output for the test case
        * explain.sh - (optional) a script that when the output from the program
          under test is run will display a helpful message about why the program
          failed
        * weight.txt - (optional) contains one floating point number that is the
          weight of this test case, defaults to 1.0 if none given.
        * category.txt - (optional) contains a string describing the rubric
          category, if omitted defaults to 'default'.

        :param p: Path to the testcase directory.
        """

        this.weight = 1.0
        this.category = "default"
        this.description = ""

        # changes to True if the test case runs successfully
        this.passed= False

        this.has_run = False

        # messages for display about the given test case
        this.messages = []

        p = pathlib.Path(p)

        this.name = p.name

        if (p / "description.txt").exists():
            with open(p / "description.txt", "r") as f:
                this.description = f.read().strip()

        if (p / "category.txt").exists():
            with open(p / "category.txt", "r") as f:
                this.category = f.read().strip()

        if (p / "weight.txt").exists():
            with open(p / "weight.txt", "r") as f:
                this.weight = float(f.read().strip())

        if (p / "input.txt").exists():
            with open(p / "input.txt", "r") as f:
                this.input = f.read()
        else:
            raise ValueError("Test case in '{}' has no input.txt".format(p))

        if (p / "output.txt").exists():
            with open(p / "output.txt", "r") as f:
                this.output = f.read().strip()
        else:
            raise ValueError("Test case in '{}' has no output.txt".format(p))

        if (p / "explain.sh").exists():
            this.explain = p / "explain.sh"
        else:
            this.explain = None

    def describe(this):
        """describe.

        This function returns a description of the test case.

        :param this:
        """

        s = "Test case '{}', category '{}', weight {:3.2f}\n".format(this.name, this.category, this.weight)
        s += '\n\t'.join(this.description.split('\n'))
        s += '\n\n'
        return s

    def run(this):
        """run.

        This test case executes the given test case and saves the results into
        the messages, has_run, and passed member variables of this class.

        Returns True if the test passed, and False otherwise. this.messages can
        be inspected for more detailed information.

        :param this:
        """

        this.has_run = True

        # create a directory for us to save our logs in
        log_dir = g.log_dir / this.name
        log_dir.mkdir()

        if not g.quiet:
            sys.stderr.write("###### {}/{}: ".format(this.category, this.weight))

        env = dict(os.environ)
        env["CSCE491_PROJECT_DIR"] = str(g.project_dir)
        env["CSCE491_CODE_DIR"] = str(g.code_dir)
        env["CSCE491_CASE_ID"] = this.name
        env["CSCE491_LOG_DIR"] = str(log_dir)

        # make clean
        if not g.quiet:
            sys.stderr.write("cleaning... ")
        builder.clean_code(log_dir/"make_clean.out", log_dir/"make_clean.err", env)

        # make
        if not g.quiet:
            sys.stderr.write("compiling... ")
        ok, errors = False, ["you should not see this"]
        try:
            ok, errors = builder.build_code(log_dir/"make.out", log_dir/"make.err", env)
        except Exception as e:
            # in this case, we want to exit because this is probably our error
            if not g.quiet:
                sys.stderr.write("\nFailed to compile your code due to an internal error:\n{}\n".format(e))
            exit(1)
        if not ok:
            # failure to compile counts a test failure
            this.messages = errors
            this.passed = False
            if not g.quiet:
                sys.stderr.write("ERROR \t\t Score: 0/{}\n".format(this.weight))
            return False

        # now run a.out
        process = subprocess.Popen(["./a.out"], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, cwd=g.code_dir, env=env)
        (output, err) = process.communicate(input=this.input.encode("utf-8"))
        exit_code = process.wait()
        output, err = output.decode(), err.decode()

        # log the results
        with open(log_dir/"run.out", "w") as f:
            f.write(output)
        with open(log_dir/"run.err", "w") as f:
            f.write(err)

        # if the exit code was nonzero, log that as an error
        if exit_code != 0:
            if not g.quiet:
                sys.stderr.write("ERROR \t\t Score: 0/{}\n".format(this.weight))
            this.messages.append("### Your code exited with a non-zero exit code {}.".format(exit_code))
            this.messages.append("### Your program's standard output was:\n{}\n".format(output))
            this.messages.append("### Your program's standard error was:\n{}\n".format(err))
            this.messages.append("### You may want to check out the log directory '{}' for full execution logs\n".format(log_dir))
            return False

        actual, expected = output.strip(), this.output.strip()
        if actual == expected:
            if not g.quiet:
                sys.stderr.write("PASS \t\t Score: {}/{}\n".format(this.weight, this.weight))
            this.passed=True
            return True

        else:

            # write a unified diff between the actual and expected output
            if not g.quiet:
                sys.stderr.write("FAIL \t\t Score: 0/{}\n".format(this.weight))
            diff = difflib.unified_diff(actual.split("\n"), expected.split("\n"), fromfile="actual", tofile="expected")
            this.messages.append("### Your program exited without error, but your output did not match what was expected.\n")
            this.messages.append("### The description of the failing test case is:\n{}\n".format(this.description))
            this.messages.append("### Difference between actual and expected output (HINT: if you don't know how to read this, look up 'unified diff format'):\n{}\n\n"
                    .format("\n".join(diff)))

            # see if there is an explainer we can run
            if this.explain is not None:

                # run it with the input being the output from our program
                process = subprocess.Popen(["sh", this.explain], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, cwd=g.code_dir)
                (explainoutput, explainerr) = process.communicate(input=output.encode("utf8"))
                exit_code = process.wait()
                output, err = explainoutput.decode(), explainerr.decode()

                with open(log_dir/"explain.out", "w") as f:
                    f.write(output)
                with open(log_dir/"explain.err", "w") as f:
                    f.write(err)

                if exit_code == 0:
                    # if the explainer dosen't know what to do, do nothing

                    this.messages.append("### A more detailed explanation of the test failure based on your specific output:\n\n{}".format(output))


def load_testcases():
    """load_testcases.

    This function loads all test cases based on g.test_case_dir.
    """

    cases = []
    for case in g.test_case_dir.iterdir():
        cases.append(TestCase(case))

    return cases

