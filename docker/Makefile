# Makefile - Makefile to build opendlv.cfsd18 using Docker-encapsulated builders.
# Copyright (C) 2016 Christian Berger
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2
# of the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.

PROJECT_NAME=opendlv.cfsd18

REPOSITORY=seresearch

IMAGE=opendlv-cfsd18-on-opendlv-lynx-on-opendlv-on-opendlv-core-on-opendavinci-on-base
IMAGE_PATH=$(REPOSITORY)/$(IMAGE)

BASE_IMAGE=opendlv-lynx-on-opendlv-on-opendlv-core-on-opendavinci-on-base
BASE_IMAGE_PATH=$(REPOSITORY)/$(BASE_IMAGE)
BASE_IMAGE_VERSION=v0.0.2

DEV_SUFFIX=-dev

IMAGE_DEV=$(IMAGE)$(DEV_SUFFIX)
IMAGE_DEV_PATH=$(REPOSITORY)/$(IMAGE_DEV)

BASE_IMAGE_DEV=$(BASE_IMAGE)$(DEV_SUFFIX)
BASE_IMAGE_DEV_PATH=$(REPOSITORY)/$(BASE_IMAGE_DEV)
BASE_IMAGE_DEV_VERSION=$(BASE_IMAGE_VERSION)

# Shall deb or rpm packages be created?
PACKAGING_ENABLED=NO

# Tools.
DOCKER=$(shell which docker)
CMAKE=$(shell which cmake)

# Determine Git-related setting.
VERSION=v$(shell head -1 ../VERSION | cut -f1 -d" ")
GIT_CURRENT_BRANCH=$(shell git rev-parse --abbrev-ref HEAD)
GIT_HASH=$(shell git rev-parse --short HEAD)
GIT_COMMITS_IN_CURRENT_BRANCH=$(shell git rev-list --count `git rev-parse --abbrev-ref HEAD`)
GIT_HAS_UNCOMMITTED_CHANGES=$(shell if [ "`git ls-files -m ..`" != "" ]; then echo "-modified"; else echo ""; fi)

# Directories.
SOURCE_TREE_PATH=$(shell dirname $(shell pwd))
BUILD_PATH_BASE=$(SOURCE_TREE_PATH)/docker/builds
BUILD_PATH=$(BUILD_PATH_BASE)/$(IMAGE)-$(GIT_CURRENT_BRANCH)
CCACHE_PATH=$(HOME)/.ccache
PACKAGES_DIR=$(BUILD_PATH)

# Set up user-related settings.
RUN_BUILD_AS=odv # OpenDaVINCI is built as odv.
RUN_BUILD_UID_AS=$(shell getent passwd `whoami` | cut -f3 -d":")

# Resulting Docker image settings.
BASE_IMAGE_GIT_HASH=$(BASE_IMAGE)-$(GIT_HASH)
BASE_IMAGE_GIT_HASH_COMMITS=$(BASE_IMAGE_GIT_HASH):$(GIT_COMMITS_IN_CURRENT_BRANCH)

BASE_IMAGE_DEV_GIT_HASH=$(BASE_IMAGE_DEV)-$(GIT_HASH)
BASE_IMAGE_DEV_GIT_HASH_COMMITS=$(BASE_IMAGE_DEV_GIT_HASH):$(GIT_COMMITS_IN_CURRENT_BRANCH)

DOCKER_NETWORK_NAME=nw_$(BASE_IMAGE_GIT_HASH)

###############################################################################

.PHONY: all buildComplete createDockerImage clean cleanAll cleanDocker removeExitedContainers removeNoneImagesFromDocker removeExistingImagesForBaseImageFromDocker removeExistingImagesForBaseImageDevFromDocker removeDockerNetworkInterfaces updateDockerBaseImage updateDockerBaseImageDev updateDockerBaseImage updateBuildComplete

all: updateBuildComplete createDockerImage

cleanAll: clean cleanDocker

clean:
	$(CMAKE) -E remove_directory $(BUILD_PATH)

cleanDocker: removeExitedContainers removeNoneImagesFromDocker removeExistingImagesForBaseImageDevFromDocker removeExistingImagesForBaseImageFromDocker removeDockerNetworkInterfaces

removeExitedContainers:
	$(DOCKER) rm $$($(DOCKER) ps -a -q) || exit 0

removeNoneImagesFromDocker:
	for i in $$($(DOCKER) images | grep none | tr -s " " ";" | cut -f3 -d";"); do $(DOCKER) rmi $$i; done

removeExistingImagesForBaseImageFromDocker:
	$(DOCKER) rmi -f `$(DOCKER) images | grep $(IMAGE) | tr -s " " ";" | cut -f3 -d";"` || exit 0

removeExistingImagesForBaseImageDevFromDocker:
	$(DOCKER) rmi -f `$(DOCKER) images | grep $(IMAGE_DEV) | tr -s " " ";" | cut -f3 -d";"` || exit 0

removeDockerNetworkInterfaces:
	$(DOCKER) network prune -f

updateDockerBaseImage: updateDockerBaseImage updateDockerBaseImageDev

updateDockerBaseImage:
	$(DOCKER) pull $(BASE_IMAGE_PATH):$(BASE_IMAGE_VERSION)

updateDockerBaseImageDev:
	$(DOCKER) pull $(BASE_IMAGE_DEV_PATH):$(BASE_IMAGE_DEV_VERSION)

###############################################################################
define compile
	$(CMAKE) -E make_directory $(BUILD_PATH)
	$(CMAKE) -E make_directory $(BUILD_PATH)/opt/$(PROJECT_NAME)
	$(CMAKE) -E make_directory $(BUILD_PATH)/$(PROJECT_NAME).build
	$(CMAKE) -E make_directory $(BUILD_PATH)/$(PROJECT_NAME).deb
	$(DOCKER) network inspect $(DOCKER_NETWORK_NAME) || $(DOCKER) network create $(DOCKER_NETWORK_NAME)
	$(DOCKER) network ls
	$(DOCKER) run -ti --rm -v $(SOURCE_TREE_PATH):/opt/$(PROJECT_NAME).sources -v $(BUILD_PATH)/$(PROJECT_NAME).build:/opt/$(PROJECT_NAME).build -v $(CCACHE_PATH):/opt/ccache -v $(BUILD_PATH)/opt/$(PROJECT_NAME):/opt/$(PROJECT_NAME) -v $(SOURCE_TREE_PATH)/docker/completeBuild.sh:/opt/completeBuild.sh -w /opt $(BASE_IMAGE_DEV_PATH):$(BASE_IMAGE_DEV_VERSION) /opt/completeBuild.sh $(RUN_BUILD_AS) $(RUN_BUILD_UID_AS) $(PACKAGING_ENABLED)
	$(DOCKER) network rm $(DOCKER_NETWORK_NAME)
	mkdir -p $(PACKAGES_DIR)/$(BASE_IMAGE) && for i in $$(find $(BUILD_PATH)/$(PROJECT_NAME).build -type f -name "*deb" | grep -v CPack); do cp $$i $(PACKAGES_DIR)/$(BASE_IMAGE); done
	touch $(BUILD_PATH)/$(PROJECT_NAME).build/build.incremental
endef

###############################################################################
# This target uses a Docker container to build the $(PROJECT_NAME) source tree using the selected Docker image.
$(BUILD_PATH)/$(PROJECT_NAME).build/build.incremental:
	$(call compile)

buildComplete:
	$(call compile)

updateBuildComplete: updateDockerBaseImage
	$(call compile)

# Dependency to allow incremental builds.
buildIncremental: $(BUILD_PATH)/$(PROJECT_NAME).build/build.incremental
	$(DOCKER) run -ti --rm -v $(SOURCE_TREE_PATH):/opt/$(PROJECT_NAME).sources -v $(BUILD_PATH)/$(PROJECT_NAME).build:/opt/$(PROJECT_NAME).build -v $(CCACHE_PATH):/opt/ccache -v $(BUILD_PATH)/opt/$(PROJECT_NAME):/opt/$(PROJECT_NAME) -v $(SOURCE_TREE_PATH)/docker/incrementalBuild.sh:/opt/incrementalBuild.sh -w /opt $(BASE_IMAGE_DEV_PATH):$(BASE_IMAGE_DEV_VERSION) /opt/incrementalBuild.sh $(RUN_BUILD_AS) $(RUN_BUILD_UID_AS)
	mkdir -p $(PACKAGES_DIR)/$(BASE_IMAGE) && for i in $$(find $(BUILD_PATH)/$(PROJECT_NAME).build -type f -name "*deb" | grep -v CPack); do cp $$i $(PACKAGES_DIR)/$(BASE_IMAGE); done

###############################################################################
# This target creates a new Docker image based on the selected Docker image using the freshly built binaries.
createDockerImage: createDockerImage createDockerImageDev

createDockerImage:
	cat $(SOURCE_TREE_PATH)/docker/Dockerfile.template.nonDev | sed -e s%BASEIMAGE%$(BASE_IMAGE_PATH)% > $(BUILD_PATH)/Dockerfile && $(CMAKE) -E remove_directory $(BUILD_PATH)/opt.nonDev && $(CMAKE) -E copy_directory $(BUILD_PATH)/opt $(BUILD_PATH)/opt.nonDev && $(CMAKE) -E remove_directory $(BUILD_PATH)/opt.nonDev/${PROJECT_NAME}/man && $(CMAKE) -E remove_directory $(BUILD_PATH)/opt.nonDev/${PROJECT_NAME}/include
	echo "${PROJECT_NAME}.build" > $(BUILD_PATH)/.dockerignore
	echo "${PROJECT_NAME}.deb" >> $(BUILD_PATH)/.dockerignore
	echo "$(BASE_IMAGE)" >> $(BUILD_PATH)/.dockerignore
	echo "opt" >> $(BUILD_PATH)/.dockerignore
	cd $(BUILD_PATH) && $(DOCKER) build -t $(REPOSITORY)/$(BASE_IMAGE_GIT_HASH_COMMITS) . | tee $(BUILD_PATH)/$(PROJECT_NAME).build/build.log || exit 1
	$(DOCKER) images | grep $$(tail -1 $(BUILD_PATH)/$(PROJECT_NAME).build/build.log | cut -f3 -d" ") | grep "latest" && exit 0 || $(DOCKER) tag $$(tail -1 $(BUILD_PATH)/$(PROJECT_NAME).build/build.log | cut -f3 -d" ") $(IMAGE_PATH):latest
	$(DOCKER) tag $(IMAGE_PATH):latest $(IMAGE_PATH):$(VERSION)

createDockerImageDev:
	cat $(SOURCE_TREE_PATH)/docker/Dockerfile.template | sed -e s%BASEIMAGE%$(BASE_IMAGE_DEV_PATH)% > $(BUILD_PATH)/Dockerfile
	echo "${PROJECT_NAME}.build" > $(BUILD_PATH)/.dockerignore
	echo "${PROJECT_NAME}.deb" >> $(BUILD_PATH)/.dockerignore
	echo "$(BASE_IMAGE)" >> $(BUILD_PATH)/.dockerignore
	echo "opt.nonDev" >> $(BUILD_PATH)/.dockerignore
	cd $(BUILD_PATH) && $(DOCKER) build -t $(REPOSITORY)/$(BASE_IMAGE_DEV_GIT_HASH_COMMITS) . | tee $(BUILD_PATH)/$(PROJECT_NAME).build/build.log || exit 1
	$(DOCKER) images | grep $$(tail -1 $(BUILD_PATH)/$(PROJECT_NAME).build/build.log | cut -f3 -d" ") | grep "latest" && exit 0 || $(DOCKER) tag $$(tail -1 $(BUILD_PATH)/$(PROJECT_NAME).build/build.log | cut -f3 -d" ") $(IMAGE_DEV_PATH):latest
	$(DOCKER) tag $(IMAGE_DEV_PATH):latest $(IMAGE_DEV_PATH):$(VERSION)

###############################################################################

pushDockerImage: createDockerImage
	$(DOCKER) push $(IMAGE_PATH):latest
	$(DOCKER) push $(IMAGE_PATH):$(VERSION)
	$(DOCKER) push $(IMAGE_DEV_PATH):latest
	$(DOCKER) push $(IMAGE_DEV_PATH):$(VERSION)

