set -e

mkdir -p build bin
cd build

cmake ..
make -j$(nproc --all)

cd ..
