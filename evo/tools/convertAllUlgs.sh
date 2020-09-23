#!/bin/bash
echo Converting all ulg files.

for filename in *.ulg; do
	echo Converting $filename
	mkdir ${filename%%.*}
	mv $filename ${filename%%.*}/
	cd ${filename%%.*}
	ulog2csv $filename
	cd ..
done
echo Done.
