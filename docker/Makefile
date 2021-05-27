.DEFAULT_GOAL := all
RELEASES := 	kinetic \
		melodic \
		noetic

.PHONY: all base rfal

jackal-%: jackal-%.Dockerfile
	docker build -t "pszenher/jackal:$*" -f "$<" .

jackal-rfal-%: jackal-rfal.Dockerfile jackal-%
	docker build -t "pszenher/jackal-rfal:$*" -f "$<" --build-arg="ROS_DISTRO=$*" .

base: $(addprefix jackal-,$(RELEASES))
rfal: $(addprefix jackal-rfal-,$(RELEASES))

all: base rfal
