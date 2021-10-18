set -e

./runbuild.sh
cd build
ctest -VV
cd ..
