#!/bin/bash
set -e

BASEDIR=$(readlink -f $(dirname $0))
cd $BASEDIR

VERSION=$(git rev-parse HEAD)
COVERDIR="$BASEDIR/build/cover-$VERSION"

function usage() {
    echo "Run unit tests and coverage reports on the codebase with optional"
    echo "http server to the report"
    echo
    echo "Usage: `basename $0` [options]";
    echo
    echo "  -h           show this message"
    echo "  -t           the type of tests to run"
    echo "  -p \$PORT     run an simple http server on \$PORT to view results"
    echo "  -v           verbose mode (set -x)"
    echo
}

while getopts ":vhp:" opt; do
  case $opt in
    t) ROVER_COVERAGE_TYPE="$OPTARG" ;;
    p) PORT="$OPTARG" ;;
    v) set -x ;;
    h)
      usage
      exit 0
      ;;
    \?)
      echo "Invalid option: -$OPTARG" >&2
      usage
      exit 1
      ;;
    :)
      echo "Option -$OPTARG requires an argument." >&2
      exit 1
      ;;
  esac
done

if [[ -v ROVER_COVERAGE_TYPE && "$ROVER_COVERAGE_TYPE" != "all" ]]; then
    nosetests ${ROVER_COVERAGE_TYPE}/ \
        --with-coverage \
        --cover-html \
        --cover-html-dir=$COVERDIR \
        --cover-erase \
        --cover-package=BinghamtonRover
else
    nosetests \
        --with-coverage \
        --cover-html \
        --cover-html-dir=$COVERDIR \
        --cover-erase \
        --cover-package=BinghamtonRover
fi


if [ "$PORT" != "" ]; then
    echo "Starting http server on :$PORT (^C to exit)"
    pushd $COVERDIR
    python -m SimpleHTTPServer $PORT || echo "Done."
    popd
fi
rm -r $COVERDIR
