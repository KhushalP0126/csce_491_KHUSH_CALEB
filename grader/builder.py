# Copyright 2021 Jason Bakos, Philip Conrad, Charles Daniels
#
# Distributed as part of the University of South Carolina CSCE491 course
# materials. Please do not redistribute without written authorization.

# This file implements code for compiling and cleaning student code under test.

import subprocess
import pathlib
import os

from . import g

def build_code(log_stderr=None, log_stdout=None, env=None):
    """build_code.

    This method compiles the student's code using the 'make' command. It
    returns (true, []) if there is no error, and (false, [list of error
    messages]) if an error occurs.

    :param log_stderr: if non-None, the standard out of running 'make' will be
    saved to this file.
    :param log_stdout: if non-None, the standard error of running 'make' will
    be saved to this file.
    """

    # run the command and get the results
    process = subprocess.Popen([g.make_command], stdout=subprocess.PIPE, stderr=subprocess.PIPE, cwd=g.code_dir, env=env)
    (output, err) = process.communicate()
    exit_code = process.wait()
    output, err = output.decode(), err.decode()

    # save the output, if desired
    if log_stdout is not None:
        with open(log_stdout, "w") as f:
            f.write(output)
    if log_stderr is not None:
        with open(log_stderr, "w") as f:
            f.write(err)

    if exit_code != 0:
        return False, [
                "Command 'make' exited with a non-zero exit code {}".format(exit_code),
                "Standard output from 'make' was:\n{}".format(output),
                "Standard error from 'make' was:\n{}".format(err),
        ]

    if not (g.code_dir / "a.out").exists():
        return False, [
                "Command 'make' exited with a zero exit code, but {}/a.out does not exist".format(g.code_dir),
                "Standard output from 'make' was:\n{}".format(output),
                "Standard error from 'make' was:\n{}".format(err),
            ]

    if not os.access(g.code_dir / "a.out", os.X_OK):
        return False, [
                "Command 'make' exited with a zero exit code, and {}/a.out exists, but it is not executable".format(g.code_dir),
                "Standard output from 'make' was:\n{}".format(output),
                "Standard error from 'make' was:\n{}".format(err),
            ]

    return True, []

def clean_code(log_stderr=None, log_stdout=None, env=None):
    """clean_code.

    This method cleans the student's code using the 'make clean' command.

    :param log_stderr: if non-None, the standard out of running 'make clean'
    will be saved to this file.
    :param log_stdout: if non-None, the standard error of running 'make clean'
    will be saved to this file.
    """

    # run the command and get the results
    process = subprocess.Popen([g.make_command, "clean"], stdout=subprocess.PIPE, stderr=subprocess.PIPE, cwd=g.code_dir, env=env)
    (output, err) = process.communicate()
    exit_code = process.wait()
    output, err = output.decode(), err.decode()

    # save the output, if desired
    if log_stdout is not None:
        with open(log_stdout, "w") as f:
            f.write(output)
    if log_stderr is not None:
        with open(log_stderr, "w") as f:
            f.write(err)

    if exit_code != 0:
        return False, [
                "Command 'make clean' exited with a non-zero exit code {}".format(exit_code),
                "Standard output from 'make' was:\n{}".format(output),
                "Standard error from 'make' was:\n{}".format(err),
        ]

    return True, []

