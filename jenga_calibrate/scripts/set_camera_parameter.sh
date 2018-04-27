#!/usr/bin/env bash
video0=/dev/video0

if [ -e $video0 ]
	then
	v4l2-ctl -d $video0 -c focus_auto=0
	v4l2-ctl -d $video0 -c focus_absolute=0
	v4l2-ctl -d $video0 -c zoom_absolute=500
	echo Successfully set camera focus for $video0.
else
	echo $video0 not found.
fi
