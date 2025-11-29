#!/bin/bash

poststr="http://api.thingspeak.com/update?key=LILP2LB48CIL4L5T"

if [ -f /tmp/field1.dat ] ; # Office Temp
	then
		read field1 < /tmp/field1.dat
		poststr="$poststr&"field1="$field1"
fi

if [ -f /tmp/field2.dat ] ; # Dining Room
	then
		read field2 < /tmp/field2.dat
		poststr="$poststr&"field2="$field2"
fi

if [ -f /tmp/field3.dat ] ; # Ensuite
	then
		read field3 < /tmp/field3.dat
		poststr="$poststr&"field3="$field3"
fi

if [ -f /tmp/field4.dat ] ; # Living Room
	then
		read field4 < /tmp/field4.dat
		poststr="$poststr&"field4="$field4"
fi

if [ -f /tmp/field5.dat ]; # Jee06 Humidity
	then
		read field5 < /tmp/field5.dat
		if [ $field5 != "0.0" ]; then
			poststr="$poststr&"field5="$field5"
		else
			echo `date "+%d/%m/%Y %X"` $field5 >> /tmp/field5.txt 
		fi
fi

if [ -f /tmp/field6.dat ] ; # Jee04
	then
		read field6 < /tmp/field6.dat
		poststr="$poststr&"field6="$field6"
fi

if [ -f /tmp/field7.dat ] ; # Jee05
	then
		read field7 < /tmp/field7.dat
		poststr="$poststr&"field7="$field7"
fi

if [ -f /tmp/field8.dat ] ; # Jee06 Temperature
	then
		read field8 < /tmp/field8.dat
		poststr="$poststr&"field8="$field8"
fi

#echo "1="$field1 "2="$field2 3=$field3 4=$field4 5=$field5 6=$field6 "7="$field7 "8="$field8

echo `date "+%d/%m/%Y %X"` $poststr >> /tmp/poststr.txt
/usr/bin/post "$poststr"
#/usr/bin/post "http://api.thingspeak.com/update?key=LILP2LB48CIL4L5T&field1=$field1&field2=$field2&field3=$field3&field4=$field4&field5=$field5&field6=$field6&field7=$field7&field8=$field8" > /dev/null 2>&1
rm /tmp/field*.dat




















