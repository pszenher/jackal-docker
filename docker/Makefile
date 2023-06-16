.DEFAULT_GOAL := all
RELEASES := 	kinetic \
		melodic \
		noetic

.PHONY: all base rfal

ubuntu-bootstrap: ubuntu-bootstrap.Dockerfile
	docker build -t "pszenher/ubuntu-bootstrap:focal" -f "$<" .

jackal-base-%: jackal-base.Dockerfile ubuntu-bootstrap
	docker build -t "pszenher/jackal-base:$*" -f "$<" .

# FIXME: need to map ros_distro -> os_distro here
jackal-ros-source-%: jackal-ros-source.Dockerfile jackal-base-focal
	docker build -t "pszenher/jackal-ros-source:$*" -f "$<" .

jackal-whaleboot-%: jackal-whaleboot.Dockerfile jackal-ros-source-%
	docker build -t "pszenher/jackal-whaleboot:$*" -f "$<" --build-arg="FINAL_IMAGE=pszenher/jackal-ros-source:$*" .

jackal-%: jackal-%.Dockerfile
	docker build -t "pszenher/jackal:$*" -f "$<" .

jackal-rfal-%: jackal-rfal.Dockerfile jackal-%
	docker build -t "pszenher/jackal-rfal:$*" -f "$<" --build-arg="ROS_DISTRO=$*" .

base: $(addprefix jackal-,$(RELEASES))
rfal: $(addprefix jackal-rfal-,$(RELEASES))

all: base rfal
