#!/bin/bash

main ()
{
  ./scripts/replace-uids.sh
  ./docker/build-images.sh
}

script_dir="$(dirname "$(readlink -f "$0")")"
pushd $script_dir > /dev/null
main $@
popd > /dev/null
