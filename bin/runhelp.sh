#!/usr/bin/zsh

# This shell script loads a web browser with the help file
# it should run from the bin directory


hfile=`pwd`/`find .. | fgrep /help/help.html`
echo hfile is $hfile

echo running help
for f in  mozilla konqueror netscape 
do
    
    if ([ -f /usr/bin/$f ] && [ -f $hfile ])
    then 
	echo executing /usr/bin/$f $hfile
	exec /usr/bin/$f $hfile &
	break
    else
	echo ERROR: Cannot find a browser or the help file!!!
    fi
done
    
echo got here
exit
echo didnot get here
