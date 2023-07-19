set -e
sudo docker run --rm -v `pwd`:/yag-slam/ quay.io/pypa/manylinux2014_x86_64 /yag-slam/build.sh
