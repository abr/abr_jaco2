#!/usr/bin/env bash
shopt -s globstar
if [[ ! -e .ci/common.sh || ! -e abr_jaco2 ]]; then
    echo "Run this script from the root directory of this repository"
    exit 1
fi
source .ci/common.sh

# This script runs the static style checks

NAME=$0
COMMAND=$1

if [[ "$COMMAND" == "install" ]]; then
    # astroid==2.2 causes an error when running pylint
    exe pip install codespell pylint gitlint "astroid<2.2.0"
elif [[ "$COMMAND" == "script" ]]; then
    exe pylint abr_jaco2 --rcfile=setup.cfg --ignore=kinova-api,jaco2_cython.cpp
    exe codespell -q 3 --skip=kinova-api,jaco2_cython.cpp
    exe shellcheck -e SC2087 .ci/*.sh
    # undo single-branch cloning
    git config --replace-all remote.origin.fetch +refs/heads/*:refs/remotes/origin/*
    git fetch origin master
    N_COMMITS=$(git rev-list --count HEAD ^origin/master)
    for ((i=0; i<N_COMMITS; i++)) do
        git log -n 1 --skip $i --pretty=%B | grep -v '^Co-authored-by:' | exe gitlint -vvv
    done
elif [[ -z "$COMMAND" ]]; then
    echo "$NAME requires a command like 'install' or 'script'"
else
    echo "$NAME does not define $COMMAND"
fi

exit "$STATUS"
