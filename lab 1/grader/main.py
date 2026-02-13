# Copyright 2021 Jason Bakos, Philip Conrad, Charles Daniels
#
# Distributed as part of the University of South Carolina CSCE491 course
# materials. Please do not redistribute without written authorization.

# This file implements the main CLI interface to the grading script.

import argparse
import pathlib
import sys
import os
import shutil
import csv
import tarfile
from io import BytesIO

from . import builder
from . import testcase

# used to store global values
from . import g

sys.path.append(str(pathlib.Path.cwd() / "utils" / "python_utils"))
from waves import Waves

labname = "lab_2023sp_spi"

def main():

    # directory where 'grade.sh' lives - if we are run under grade.sh, this
    # will be our CWD
    g.project_dir = pathlib.Path.cwd()

    # directory where student code lives
    g.code_dir = g.project_dir / "code"

    # directory where test cases live
    g.test_case_dir = g.project_dir / "test_cases"

    # directory where test case logs should be written
    g.log_dir = g.project_dir  / "logs"


    parser = argparse.ArgumentParser("CSCE491 grading script. Should be run through grade.sh.")

    parser.add_argument("--rubric", "-R", default=False, action='store_true', help="Display a rubric based on the available test cases and then exit without running any tests.")

    parser.add_argument("--quiet", "-q", default=False, action='store_true', help="Don't display any output unless --total is passed, in which case it is the only output.")

    parser.add_argument("--total", "-t", default=False, action='store_true', help="Display the total score as a floating point number in 0.0 ... 1.0 as the final line on standard out, followed by a '\t' and the user id. Implies --continue.")

    parser.add_argument("--continue", "-c", default=False, action='store_true', help="Continue running even if a test case fails.")

    parser.add_argument("--noscore", "-n", default=False, action='store_true', help="Don't display a score summary.")

    parser.add_argument("--only", "-o", default=[], nargs="*", help="Only run test cases passed to this flag, ignoring all others. Implies --noscore. Can accept multiple test cases to run.")

    parser.add_argument("--omit", "-O", default=[], nargs="*", help="Omit test cases passed to this flag, running all others. Implies --noscore. Can accept multiple test cases to omit.")

    parser.add_argument("--text2vcd", nargs=2, type=pathlib.Path, metavar=("INPUT", "OUTPUT"), help="Instead of grading, convert the first argument to --text2vcd to a VCD file, writing out to the path on the second argument. The input file should be in the text format used for this course.")

    parser.add_argument("--vcd2text", nargs=2, type=pathlib.Path, metavar=("INPUT", "OUTPUT"), help="Instead of grading, convert the first argument to --text2vcd to a text file in the format used for this course, writing out to the path on the second argument. The in put file needs to be in VCD format.")

    parser.add_argument("--code_dir", "-C", type=pathlib.Path, default=g.code_dir, help="Override directory where code to be graded is stored. (default: ./code")

    parser.add_argument("--case_dir", "-T", type=pathlib.Path, default=g.test_case_dir, help="Override directory where test cases are stored. (default: ./test_cases")

    parser.add_argument("--log_dir", "-l", type=pathlib.Path, default=g.log_dir, help="Override directory where execution logs are stored. (default: ./logs")

    parser.add_argument("--userid", "-u", default=None, help="Override user ID, which is normally read from code_dir/userid.txt.")

    parser.add_argument("--pack", "-p", action="store_true", default=False, help="Rather than grading, pack up the code folder for submitting.")

    args = parser.parse_args()

    # allow directory overrides if given
    g.code_dir = args.code_dir.resolve()
    g.test_case_dir = args.case_dir.resolve()
    g.log_dir = args.log_dir.resolve()

    # text2vcd utility
    if args.text2vcd != None:
        infile = ""
        with open(args.text2vcd[0], "r") as f:
            infile = f.read()
        w = Waves()
        w.loadText(infile)
        with open(args.text2vcd[1], "w") as f:
            f.write(w.toVCD())

        exit(0)

    # vcd2text utility
    if args.vcd2text != None:
        infile = ""
        with open(args.vcd2text[0], "r") as f:
            infile = f.read()
        w = Waves()
        w.loadVCD(infile)
        with open(args.vcd2text[1], "w") as f:
            f.write(w.toText())

        exit(0)

    # try to get the user ID from the CLI, if not try to get it from the file
    userid = args.userid
    if userid is None:
        userid_txt = g.code_dir / "userid.txt"
        if userid_txt.exists():
            with open(userid_txt, "r") as f:
                userid = f.read().strip()

    # we need to have a user ID -- note that for testing purposes, you can just
    # use -u test or something like that to avoid having to create the file
    if userid is None:
        sys.stderr.write("Error: could not determine user ID. Please create the file {}/userid.txt and write your user ID there. For example, your school email is 'jsmith@email.sc.edu', please write 'jsmith' in userid.txt.\n".format(g.code_dir))
        exit(1)

    g.quiet = args.quiet
    if args.quiet:
        # don't display scores if we are being quiet
        args.noscore = True

    if len(args.only) + len(args.omit) > 0:
        # also, if we are only running some test cases, we don't have enough
        # information to display a score
        args.noscore = True

    if args.total:
        # --total implies --continue
        args.__setattr__("continue", True)

    # configured MAKE variable if the environment defines one
    g.make_command = "make"
    if "MAKE" in os.environ:
        g.make_command=os.environ["MAKE"]
        if not args.quiet:
            sys.stderr.write("Picked up non-default MAKE setting: '{}'\n".format(g.make_command))

    # clean up old logs
    if g.log_dir.exists():
        shutil.rmtree(g.log_dir)
    g.log_dir.mkdir()

    # pack up submission if requested
    if args.pack:
        # make clean
        sys.stderr.write("cleaning code... ")
        builder.clean_code()
        sys.stderr.write("DONE\n")

        # tar up the project
        sys.stderr.write("packing submission... ")
        outfile = g.project_dir / (labname + "_" + userid + ".tar.gz")
        with tarfile.open(str(outfile), "w:gz") as tar:

            # add in the c ode folder
            tar.add(g.code_dir, arcname="code_"+userid)

            # add userid.txt explicitly, this is done so that if -u is used,
            # we still know what the userid was when we unpack it, that way
            # --total will still give correct output
            s = BytesIO()
            s.write(userid.encode("utf8"))
            s.seek(0)
            i = tarfile.TarInfo(name="code_"+userid+"/userid.txt")
            i.size = len(userid.encode("utf8"))
            tar.addfile(i, s)

        sys.stderr.write("DONE\n")
        sys.stderr.write("Your submission file has been saved to: '{}'\n".format(outfile))
        exit(0)

    if not args.quiet:
        sys.stderr.write("Loading test cases... \n")
    cases = testcase.load_testcases()
    if not args.quiet:
        sys.stderr.write("Loaded {} test cases.\n".format(len(cases)))

    # enumerate all rubric categories
    categories = []
    max_points = 0
    for case in cases:
        categories.append(case.category)
        max_points += case.weight
    categories=sorted(list(set(categories)))

    if args.rubric:
        # display each one
        for cat in categories:
            # find all members of the category
            members = []
            member_points = 0
            for case in cases:
                if case.category == cat:
                    members.append(case)
                    member_points += case.weight

            # display them all
            print("Rubric category: {}, {} test cases, {} points".format(cat, len(members), member_points))
            for m in members:
                print("\t{} - {} points".format(m.name, m.weight))

        print("\nMax points across all categories: {}".format(max_points))
        print("NOTE: this rubric is only based on the provided test cases, check the lab sheet for any additional items that might not be included here!")

        exit(0)

    # if we weren't told to do something else, go ahead and run all the test
    # cases
    failed = 0
    for case in sorted(cases, key=lambda c: c.name):
        if case.name in args.omit:
            continue

        if (len(args.only) > 0) and (case.name not in args.only):
            continue

        if not case.run():
            for msg in case.messages:
                if not args.quiet:
                    sys.stderr.write("{}\n".format(msg))
            failed += 1

            if not args.__dict__["continue"]:
                exit(1)

    if not args.noscore:
        print("###### Score summary: ")

    # generate the rubric
    earned_points = 0
    for cat in categories:
        # find all members of the category
        members = []
        member_points = 0
        member_max = 0
        for case in sorted(cases, key=lambda c: c.name):
            if case.category == cat:
                members.append(case)
                member_max += case.weight
                if case.passed:
                    member_points += case.weight
                    earned_points += case.weight

        # display them all
        if not args.noscore:
            print("Rubric category: {}, {} test cases, {}/{} points".format(cat, len(members), member_points, member_max))
            for m in members:
                print("\t{} - {}/{} points".format(m.name, m.weight * float(m.passed), m.weight))

    if not args.noscore:
        print("Estimated total score: {}/{} ({:3.2f}%)".format(earned_points, max_points, 100.0*earned_points/max_points))
        print("NOTE: this is only an estimated score based on the provided example test cases. It may or may not reflect your actual score on the assignment. However, you can expect your true score will be no higher than the estimated score.")

    if args.total:
        print("{:1.3f}\t{}".format(float(earned_points)/max_points, userid))


if __name__ == "__main__":
    main()
