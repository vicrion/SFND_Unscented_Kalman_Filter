SHELL := /bin/bash

BASE.DIR=$(PWD)
COMMON.DIR=$(realpath $(BASE.DIR)/..)
DOWNLOADS.DIR=$(COMMON.DIR)/downloads
INSTALLED.HOST.DIR=$(COMMON.DIR)/installed.host

deps: eigen.fetch eigen.build flann.fetch flann.build

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

# https://github.com/boostorg/boost/releases/download/boost-1.86.0/boost-1.86.0-b2-nodocs.tar.gz
BOOST.VERSION=1.86.0
BOOST.ARCHIVE=boost-$(BOOST.VERSION)-b2-nodocs.tar.gz
BOOST.URL=https://github.com/boostorg/boost/releases/download/boost-$(BOOST.VERSION)/$(BOOST.ARCHIVE)
BOOST.DIR=$(DOWNLOADS.DIR)/boost-$(BOOST.VERSION)
BOOST.BUILD=$(DOWNLOADS.DIR)/build.boost-$(BOOST.VERSION)
boost.fetch: .FORCE
	rm -rf $(BOOST.DIR) && rm -f $(BOOST.ARCHIVE)
	cd $(DOWNLOADS.DIR) && wget $(BOOST.URL) -O $(BOOST.ARCHIVE) && tar xvf $(BOOST.ARCHIVE)

boost.build: .FORCE
	cd $(BOOST.DIR) && ./bootstrap.sh --prefix=$(INSTALLED.HOST.DIR) && ./b2 stage threading=multi link=shared --without-python && ./b2 install threading=multi link=shared --without-python

.FORCE:

