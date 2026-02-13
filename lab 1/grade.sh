#!/bin/sh

cd "$(dirname "$0")"

set -e

# If you need to use different binaries than the default
# python3/virtualenv/make on your system, use environment variables to override
# them. If you do this, remember that we won't be using your overridden values
# during grading in the Linux lab.
#
# For example, say you wanted to use a custom Python 3 binary, you could
# use the command 'PYTHON=/my/custom/python/binary sh grade.sh'.

if [ -z "$PYTHON" ] ; then
	PYTHON=python3
fi

if [ -z "$VIRTUALENV" ] ; then
	VIRTUALENV=virtualenv
fi

if [ -z "$MAKE" ] ; then
	MAKE=make
fi

set -u

if [ ! -x "$(which "$PYTHON" )" ] ; then
	echo "ERROR: no '$PYTHON' in PATH" 1>&2
	exit 1
fi

#if [ ! -x "$(which "$VIRTUALENV")" ] ; then
#	echo "ERROR: no '$VIRTUALENV' in PATH" 1>&2
#	exit 1
#fi

if [ ! -x "$(which "$MAKE")" ] ; then
	echo "ERROR: no '$MAKE' in PATH" 1>&2
	exit 1
fi

if [ ! -f ./.setup_done ] ; then
	echo "Looks like you have not run 'grade.sh' before, running first time setup... " 1>&2
	rm -rf ./.venv
	python3 -m virtualenv --python "$PYTHON" ./.venv 
	. ./.venv/bin/activate
	./.venv/bin/pip3 install pyDigitalWaveTools==1.0
	echo "First time setup done!"
	touch .setup_done
fi

. ./.venv/bin/activate

PYTHON="$(pwd)/.venv/bin/python3"


"$PYTHON" -m grader "$@"
