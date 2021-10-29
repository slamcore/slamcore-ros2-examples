#-------------------------------------------------------------------------------
# Copyright 2021 SLAMcore Ltd
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its contributors
#    may be used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#-------------------------------------------------------------------------------

# configuration ----------------------------------------------------------------
define announce
	@echo "========================================================================="
	@echo $(1)
	@echo "========================================================================="
	@echo
endef

SHELL:=/bin/bash
THIS_FILE:=$(lastword $(MAKEFILE_LIST))
THIS_DIR:=$(dir $(abspath ${THIS_FILE}))
THIS_REPONAME:=$(shell basename ${THIS_DIR})
DOCKER_NAME:=slamcore_ros2_nav
DOCKER_IMAGE_NAME:=${DOCKER_NAME}:latest
DOCKERFILE_PATH:=${THIS_DIR}/Dockerfile
DOCKER_USER:=slamcore
ROS_WS:=/home/${DOCKER_USER}/ros_ws

LOGIN_CMD:=bash /entrypoint.sh

help:    ## Show this help message.
	@echo "Helper script to manage the docker container for the SLAMcore - ROS2"
	@echo "examples." "You can use this script to build the image and container"
	@echo "(build), start and stop the container (run, stop), or spawn additional"
	@echo "terminals in the running container (login)"
	@echo
	@echo "Overview"
	@echo "--------"
	@fgrep -h "##" $(MAKEFILE_LIST) | fgrep -v fgrep | sed -e 's/\\$$//' | sed -e 's/##//'

# targets ----------------------------------------------------------------------

.PHONY: prereqs
prereqs: ## Check the prerequisites for the host system at hand.
	@${THIS_DIR}/share/check-groups.sh

.PHONY: build
build:   ## Build a docker image using the slamcore_ros2.deb package.
         ## Example: SLAMCORE_DEB=/path/to/slamcore_ros.deb make build
	$(call announce,"Building SLAMcore ROS2 examples docker image - name -> ${DOCKER_NAME}")

	@if test -z ${SLAMCORE_DEB}; then \
		echo 'You have to set the SLAMCORE_DEB environment variable before "make build".' && \
		echo 'Exiting...' && \
		exit 1; \
		fi

	tempdir=$(shell mktemp --directory --suffix "-slamcore") && \
	cp ${SLAMCORE_DEB} $$tempdir/slamcore_ros2.deb && \
	cp ${THIS_DIR}/share/entrypoint.sh $$tempdir/entrypoint.sh && \
	cp ${THIS_DIR}/share/add-groups.sh $$tempdir/add-groups.sh && \
	docker build \
		--build-arg SLAMCORE_DEB=slamcore_ros2.deb \
		--build-arg GID=$(shell id -g) \
		--build-arg UID=$(shell id -u) \
		--build-arg ADDITIONAL_GROUPS=$(shell cut -d: -f3 < <(getent group video)),$(shell cut -d: -f3 < <(getent group dialout)) \
		-t ${DOCKER_NAME} \
		-f ${DOCKERFILE_PATH} $$tempdir && \
	rm -rf $$tempdir


.PHONY: run
run:     ## Start the container for slamcore-ros2-examples.
	$(call announce,"Starting the container...")

	docker_cmd="docker run"; \
	$$docker_cmd \
		--volume ${THIS_DIR}:${ROS_WS}/${THIS_REPONAME} \
		--volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
		--volume /dev/:/dev/ \
		--volume /var/run/dbus/:/var/run/dbus/ \
		--volume ${HOME}/.bash_history:/home/${DOCKER_USER}/.bash_history \
		--network host  \
		--ipc=host \
		--privileged \
		--env DISPLAY=${DISPLAY} \
		--env=QT_X11_NO_MITSHM=1 \
		-it --name ${DOCKER_NAME} ${DOCKER_IMAGE_NAME}


login:   ## Get a console in the docker container from the current terminal - must be already started for this to work.
	$(call announce,"Connecting to container -> ${DOCKER_NAME}")
	docker container start ${DOCKER_NAME}
	docker exec -it ${DOCKER_NAME} ${LOGIN_CMD}

.PHONY: stop
stop:    ## Stop and remove the running container.
	-docker stop ${DOCKER_NAME}
	-docker container rm ${DOCKER_NAME}
