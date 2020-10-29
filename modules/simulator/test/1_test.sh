#########################################################################
# File Name: testOneVessel.sh
# Author: FU Zhenqiu
# mail: fuzhenqiu0810@gmail.com
# Created Time: 2020年10月29日 星期四 20时32分49秒
#########################################################################
#!/bin/bash
rm -r build
mkdir build
cd build
cmake ..
make
echo "compile finish"
./testsimulator
