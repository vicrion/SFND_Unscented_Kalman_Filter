SHELL := /bin/bash

BASE.DIR=$(PWD)
COMMON.DIR=$(realpath $(BASE.DIR)/..)
DOWNLOADS.DIR=$(COMMON.DIR)/downloads
INSTALLED.HOST.DIR=$(COMMON.DIR)/installed.host

deps: eigen.fetch eigen.build

EIGEN.VERSION=3.4.0
EIGEN.ARCHIVE=eigen-$(EIGEN.VERSION).tar.gz
EIGEN.URL=https://gitlab.com/libeigen/eigen/-/archive/$(EIGEN.VERSION)/$(EIGEN.ARCHIVE)
EIGEN.DIR=$(DOWNLOADS.DIR)/eigen-$(EIGEN.VERSION)
EIGEN.BUILD=$(DOWNLOADS.DIR)/build.eigen-$(EIGEN.VERSION)
eigen.fetch: .FORCE
	rm -rf $(DOWNLOADS.DIR)/$(EIGEN.ARCHIVE) && mkdir -p $(DOWNLOADS.DIR)
	cd $(DOWNLOADS.DIR) && wget $(EIGEN.URL) && tar xf $(EIGEN.ARCHIVE)

eigen.build: .FORCE	
	rm -rf $(EIGEN.BUILD) && mkdir -p $(EIGEN.BUILD) && mkdir -p $(INSTALLED.HOST.DIR)
	cd $(EIGEN.BUILD) && cmake -DCMAKE_INSTALL_PREFIX=$(INSTALLED.HOST.DIR) $(EIGEN.DIR) && make -j$(J) && make install

FLANN.VERSION=1.9.2
FLANN.ARCHIVE=flann-$(FLANN.VERSION).tar.gz
FLANN.URL=https://github.com/mariusmuja/flann/archive/$(FLANN.VERSION).tar.gz
FLANN.DIR=$(DOWNLOADS.DIR)/flann-$(FLANN.VERSION)
FLANN.BUILD=$(DOWNLOADS.DIR)/build.flann
flann.fetch: .FORCE
	rm -rf $(FLANN.DIR) && rm -rf $(DOWNLOADS.DIR)/$(FLANN.ARCHIVE) && mkdir -p $(DOWNLOADS.DIR)
	cd $(DOWNLOADS.DIR) && wget $(FLANN.URL) -O $(FLANN.ARCHIVE) && tar xf $(FLANN.ARCHIVE)

flann.build: .FORCE	
	rm -rf $(FLANN.BUILD) && mkdir -p $(FLANN.BUILD)
	cd $(FLANN.BUILD) && cmake -DCMAKE_INSTALL_PREFIX=$(INSTALLED.HOST.DIR) -DCMAKE_INCLUDE_PATH=$(INSTALLED.HOST.DIR)/include -DCMAKE_LIBRARY_PATH=$(INSTALLED.HOST.DIR)/lib -DBUILD_PYTHON_BINDINGS=OFF -DBUILD_MATLAB_BINDINGS=OFF -DBUILD_EXAMPLES=OFF -DBUILD_TESTS=OFF -DBUILD_DOC=OFF $(FLANN.DIR) && make -j$(J) install

.FORCE:

