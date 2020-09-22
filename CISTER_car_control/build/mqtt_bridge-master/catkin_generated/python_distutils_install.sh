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

echo_and_run cd "/home/enio/OneDrive/Cister/ROS/Inline/CISTER_car_control/src/mqtt_bridge-master"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/enio/OneDrive/Cister/ROS/Inline/CISTER_car_control/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/enio/OneDrive/Cister/ROS/Inline/CISTER_car_control/install/lib/python2.7/dist-packages:/home/enio/OneDrive/Cister/ROS/Inline/CISTER_car_control/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/enio/OneDrive/Cister/ROS/Inline/CISTER_car_control/build" \
    "/usr/bin/python2" \
    "/home/enio/OneDrive/Cister/ROS/Inline/CISTER_car_control/src/mqtt_bridge-master/setup.py" \
    build --build-base "/home/enio/OneDrive/Cister/ROS/Inline/CISTER_car_control/build/mqtt_bridge-master" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/enio/OneDrive/Cister/ROS/Inline/CISTER_car_control/install" --install-scripts="/home/enio/OneDrive/Cister/ROS/Inline/CISTER_car_control/install/bin"
