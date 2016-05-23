#!/usr/bin/zsh

# This shell script loads a web browser with the help file
# it should run from the bin directory

for f in mozilla konqueror netscape
do
    if [ -e /usr/bin/$f ]
    then 
	exec /usr/bin/$f `pwd`/../help/help.html
	exit
    fi
done
    
