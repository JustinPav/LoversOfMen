# generated from colcon_bash/shell/template/package.bash.em

# This script extends the environment for this package.

# a bash script is able to determine its own path if necessary
if [ -z "$COLCON_CURRENT_PREFIX" ]; then
  # the prefix is two levels up from the package specific share directory
  _colcon_package_bash_COLCON_CURRENT_PREFIX="$(builtin cd "`dirname "${BASH_SOURCE[0]}"`/../.." > /dev/null && pwd)"
else
  _colcon_package_bash_COLCON_CURRENT_PREFIX="$COLCON_CURRENT_PREFIX"
fi

# function to source another script with conditional trace output
# first argument: the path of the script
# additional arguments: arguments to the script
_colcon_package_bash_source_script() {
  if [ -f "$1" ]; then
    if [ -n "$COLCON_TRACE" ]; then
      echo "# . \"$1\""
    fi
    . "$@"
  else
    echo "not found: \"$1\"" 1>&2
  fi
}

# source sh script of this package
<<<<<<< HEAD:install/perception/share/perception/package.bash
_colcon_package_bash_source_script "$_colcon_package_bash_COLCON_CURRENT_PREFIX/share/perception/package.sh"
=======
_colcon_package_bash_source_script "$_colcon_package_bash_COLCON_CURRENT_PREFIX/share/integration/package.sh"
>>>>>>> 89f9c3bbb5a0dfe12bee0bb73fd3ac8adf4ba92b:install/integration/share/integration/package.bash

unset _colcon_package_bash_source_script
unset _colcon_package_bash_COLCON_CURRENT_PREFIX
