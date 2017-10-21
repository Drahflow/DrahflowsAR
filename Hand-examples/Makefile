%: %.cpp
	# g++ -fprofile-arcs -ftest-coverage -O4 -W -Wall -Wextra -Werror -ggdb -lopencv_highgui -lopencv_core -I/usr/include/opencv2 -S $<
	g++ -fprofile-arcs -ftest-coverage -O4 -W -Wall -Wextra -Werror -ggdb -lopencv_highgui -lopencv_core -I/usr/include/opencv2 -o $@ $<

%-asm: %.s
	g++ -fprofile-arcs -ftest-coverage -O4 -W -Wall -Wextra -Werror -ggdb -lopencv_highgui -lopencv_core -I/usr/include/opencv2 -o $@ $<
