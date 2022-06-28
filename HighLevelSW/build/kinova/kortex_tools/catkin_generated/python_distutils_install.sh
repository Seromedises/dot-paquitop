#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/paquitop/dot-paquitop/HighLevelSW/src/kinova/kortex_tools"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/paquitop/dot-paquitop/HighLevelSW/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/paquitop/dot-paquitop/HighLevelSW/install/lib/python2.7/dist-packages:/home/paquitop/dot-paquitop/HighLevelSW/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/paquitop/dot-paquitop/HighLevelSW/build" \
    "/usr/bin/python2" \
    "/home/paquitop/dot-paquitop/HighLevelSW/src/kinova/kortex_tools/setup.py" \
     \
    build --build-base "/home/paquitop/dot-paquitop/HighLevelSW/build/kinova/kortex_tools" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/paquitop/dot-paquitop/HighLevelSW/install" --install-scripts="/home/paquitop/dot-paquitop/HighLevelSW/install/bin"
