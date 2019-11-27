#!/bin/bash

main ()
{
  ./dev/build-image.sh
}

script_dir="$(dirname "$(readlink -f "$0")")"
pushd $script_dir > /dev/null
main $@
popd > /dev/null
