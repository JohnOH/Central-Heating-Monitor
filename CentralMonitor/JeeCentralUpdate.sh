#!/bin/bash
args=("$@")	# Assign to array
let "t=${args[0]}"
if [ -f /tmp/JeeCentralUpdate ] ;
	then
		read postKey ts lastAvg r1 r2 r3 r4 r5 r6 r7 r8 r9 overflow < /tmp/JeeCentralUpdate
	else
		let "postKey=ts=0"
		let "lastAvg = 0"
		let "r1=r2=r3=r4=r5=r6=r7=r8=r9=$t"
fi
printf -v now '%(%s)T' -2	# Epoc when shell started
if (( $now > $ts+599 )) ; then	# Max one sample per 10 minutes
	let "Avg=$t+$r1+$r2+$r3+$r4+$r5+$r6+$r7+$r8+$r9+5"
	let "Avg=$Avg/10"
	if [ "$lastAvg" -ne "$Avg" ]; then
		let "postKey++"
		let "postKey = ( $postKey%16 )+240"
		echo "$postKey $now $Avg $t $r1 $r2 $r3 $r4 $r5 $r6 $r7 $r8 $r9" > /tmp/JeeCentralUpdate
		if [ "$Avg" -gt "18" ]; then
			/usr/bin/jee 17,212,$postKey,$postKey,100,4600p	*Script:JeeCentralUpdate 46.0
		elif [ "$Avg" -eq "18" ]; then
			/usr/bin/jee 17,212,$postKey,$postKey,100,4700p	*Script:JeeCentralUpdate 47.0
		elif [ "$Avg" -eq "17" ]; then
			/usr/bin/jee 17,212,$postKey,$postKey,100,4800p	*Script:JeeCentralUpdate 48.0
		elif [ "$Avg" -eq "16" ]; then
			/usr/bin/jee 17,212,$postKey,$postKey,100,4900p	*Script:JeeCentralUpdate 49.0
		elif [ "$Avg" -eq "15" ]; then
			/usr/bin/jee 17,212,$postKey,$postKey,100,5100p	*Script:JeeCentralUpdate 51.0
		elif [ "$Avg" -eq "14" ]; then
			/usr/bin/jee 17,212,$postKey,$postKey,100,5200p	*Script:JeeCentralUpdate 52.0
		elif [ "$Avg" -eq "13" ]; then
			/usr/bin/jee 17,212,$postKey,$postKey,100,5220p	*Script:JeeCentralUpdate 52.2
		elif [ "$Avg" -eq "12" ]; then
			/usr/bin/jee 17,212,$postKey,$postKey,100,5440p	*Script:JeeCentralUpdate 54.4
		elif [ "$Avg" -eq "11" ]; then
			/usr/bin/jee 17,212,$postKey,$postKey,100,5540p	*Script:JeeCentralUpdate 56.2
		elif [ "$Avg" -eq "10" ]; then
			/usr/bin/jee 17,212,$postKey,$postKey,100,5600p	*Script:JeeCentralUpdate 56.0
		elif [ "$Avg" -eq "9" ]; then
			/usr/bin/jee 17,212,$postKey,$postKey,100,5700p	*Script:JeeCentralUpdate 57.0
		elif [ "$Avg" -eq "8" ]; then	# -2
			/usr/bin/jee 17,212,$postKey,$postKey,100,5780p	*Script:JeeCentralUpdate 57.8
		elif [ "$Avg" -eq "7" ]; then	# -4
			/usr/bin/jee 17,212,$postKey,$postKey,100,5880p	*Script:JeeCentralUpdate 58.8
		elif [ "$Avg" -eq "6" ]; then
			/usr/bin/jee 17,212,$postKey,$postKey,100,5980p	*Script:JeeCentralUpdate 59.8
		fi
	else
		echo "$postKey $now $Avg $t $r1 $r2 $r3 $r4 $r5 $r6 $r7 $r8 $r9" > /tmp/JeeCentralUpdate
	fi
fi
#