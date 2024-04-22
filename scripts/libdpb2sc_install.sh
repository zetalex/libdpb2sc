#!/bin/bash
cd $1/src/
cp *.h $3/usr/include
cp $1/Release/libdpb2sc.so $3/usr/lib

cd $1/Release/
scp libdpb2sc.so root@$2:/usr/lib

echo "Library installed succesfully"

exit
