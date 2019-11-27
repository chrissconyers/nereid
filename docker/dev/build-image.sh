#!/bin/bash

image_name=nereid-dev

main ()
{
  build_image
}

build_image()
{
  echo "building $image_name image"
  docker build -t $image_name \
    -f Dockerfile .
}

script_dir="$(dirname "$(readlink -f "$0")")"
pushd $script_dir > /dev/null
main $@
popd > /dev/null
