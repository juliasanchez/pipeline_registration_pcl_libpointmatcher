FILE="$1"
awk '{if (NR!=1) {print}}' "$1" >"${FILE%%.*}"_temp.csv 
awk 'END{printf("# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nWIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA ascii\n",NR,NR)}' "${FILE%%.*}"_temp.csv >entete.txt
awk -F"," '{printf("%g %g %g %g\n",$2,$3,$4,$5)}' "${FILE%%.*}"_temp.csv  > "${FILE%%.*}"_temp1.csv 
cat entete.txt >"${FILE%%.*}".pcd
cat "${FILE%%.*}"_temp1.csv >>"${FILE%%.*}".pcd
rm "${FILE%%.*}"_temp1.csv 
rm "${FILE%%.*}"_temp.csv 

