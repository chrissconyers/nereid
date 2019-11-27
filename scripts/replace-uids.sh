#!/bin/bash

main ()
{
  USER_UID=$(id -u)
  USER_GID=$(id -g)
  sed_cmd="s/#USER_UID#/$USER_UID/;s/#USER_GID#/$USER_GID/"
  root_dir=$script_dir/..

  replace_uids_in_file $root_dir/.devcontainer/Dockerfile.template $root_dir/.devcontainer/Dockerfile
  replace_uids_in_file $root_dir/.devcontainer/devcontainer.json.template $root_dir/.devcontainer/devcontainer.json
}

replace_uids_in_file()
{
  src=$1
  dst=$2
  echo "updating uids in $dst"
  if [ -f $src ]; then
    cat $src | sed $sed_cmd > $dst
  else
    echo "ERROR: $src file does not exist"
    exit 1
  fi
}

script_dir="$(dirname "$(readlink -f "$0")")"
pushd $script_dir > /dev/null
main $@
popd > /dev/null
