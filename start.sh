export GNUMAKEFLAGS=--no-print-directory

mkdir -p build
cd build

if [ ! -f ./Makefile ]; then
    cmake ..
fi
make

export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:$PWD/build
export GAZEBO_RESOURCE_PATH=/usr/share/gazebo-11:/usr/share/gazebo_models:$PWD

if ! pgrep -x "gazebo" > /dev/null; then
    ( gazebo --verbose ../csc376_assigment.world > /dev/null 2>&1 & )
    sleep 5
fi

./csc376-assigment
cd ..