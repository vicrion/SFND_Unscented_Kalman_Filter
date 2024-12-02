SHELL := /bin/bash

BASE.DIR=$(PWD)
COMMON.DIR=$(realpath $(BASE.DIR)/..)
DOWNLOADS.DIR=$(COMMON.DIR)/downloads
INSTALLED.HOST.DIR=$(COMMON.DIR)/installed.host

EIGEN.VERSION=3.4.0
EIGEN.ARCHIVE=eigen-$(EIGEN.VERSION).tar.gz
EIGEN.URL=https://gitlab.com/libeigen/eigen/-/archive/$(EIGEN.VERSION)/$(EIGEN.ARCHIVE)
EIGEN.DIR=$(DOWNLOADS.DIR)/eigen-$(EIGEN.VERSION)
EIGEN.BUILD=$(DOWNLOADS.DIR)/build.eigen-$(EIGEN.VERSION)

eigen.fetch: .FORCE
	rm -rf $(DOWNLOADS.DIR)/$(EIGEN.ARCHIVE) && mkdir -p $(DOWNLOADS.DIR)
	cd $(DOWNLOADS.DIR) && wget $(EIGEN.URL) && tar xf $(EIGEN.ARCHIVE)

# /home/vika/github/personal/installed.host
eigen.build: .FORCE	
	rm -rf $(EIGEN.BUILD) && mkdir -p $(EIGEN.BUILD) && mkdir -p $(INSTALLED.HOST.DIR)
	cd $(EIGEN.BUILD) && cmake -DCMAKE_INSTALL_PREFIX=$(INSTALLED.HOST.DIR) $(EIGEN.DIR) && make -j$(J) && make install

.FORCE:

