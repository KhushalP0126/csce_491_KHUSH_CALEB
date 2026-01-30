# Copyright 2021 Jason Bakos, Philip Conrad, Charles Daniels
#
# Distributed as part of the University of South Carolina CSCE491 course
# materials. Please do not redistribute without written authorization.

a.out: requirements
	echo '#!/bin/sh' > $@
	echo 'python3 main.py' >> $@
	echo 'exit $$?' >> $@
	chmod +x $@
.PHONY: a.out

requirements:
	if -e ./requirements.txt ; then pip install -r ./requirements.txt ; fi
.PHONY: requirements

clean:
	rm -f a.out
.PHONY: clean

