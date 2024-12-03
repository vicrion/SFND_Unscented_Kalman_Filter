SHELL := /bin/bash

BASE.DIR=$(PWD)
COMMON.DIR=$(realpath $(BASE.DIR)/..)
DOWNLOADS.DIR=$(COMMON.DIR)/downloads
INSTALLED.HOST.DIR=$(COMMON.DIR)/installed.host

deps: eigen flann boost vtk pcl

EIGEN.VERSION=3.4.0
EIGEN.ARCHIVE=eigen-$(EIGEN.VERSION).tar.gz
EIGEN.URL=https://gitlab.com/libeigen/eigen/-/archive/$(EIGEN.VERSION)/$(EIGEN.ARCHIVE)
EIGEN.DIR=$(DOWNLOADS.DIR)/eigen-$(EIGEN.VERSION)
EIGEN.BUILD=$(DOWNLOADS.DIR)/build.eigen-$(EIGEN.VERSION)
eigen: eigen.fetch eigen.build
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
flann: flann.fetch flann.build
flann.fetch: .FORCE
	rm -rf $(FLANN.DIR) && rm -rf $(DOWNLOADS.DIR)/$(FLANN.ARCHIVE) && mkdir -p $(DOWNLOADS.DIR)
	cd $(DOWNLOADS.DIR) && wget $(FLANN.URL) -O $(FLANN.ARCHIVE) && tar xf $(FLANN.ARCHIVE)

flann.build: .FORCE	
	rm -rf $(FLANN.BUILD) && mkdir -p $(FLANN.BUILD)
	cd $(FLANN.BUILD) && cmake -DCMAKE_INSTALL_PREFIX=$(INSTALLED.HOST.DIR) -DCMAKE_INCLUDE_PATH=$(INSTALLED.HOST.DIR)/include -DCMAKE_LIBRARY_PATH=$(INSTALLED.HOST.DIR)/lib -DBUILD_PYTHON_BINDINGS=OFF -DBUILD_MATLAB_BINDINGS=OFF -DBUILD_EXAMPLES=OFF -DBUILD_TESTS=OFF -DBUILD_DOC=OFF $(FLANN.DIR) && make -j$(J) install

# https://github.com/boostorg/boost/releases/download/boost-1.86.0/boost-1.86.0-b2-nodocs.tar.gz
BOOST.VERSION=1.85.0
BOOST.ARCHIVE=boost-$(BOOST.VERSION)-b2-nodocs.tar.gz
BOOST.URL=https://github.com/boostorg/boost/releases/download/boost-$(BOOST.VERSION)/$(BOOST.ARCHIVE)
BOOST.DIR=$(DOWNLOADS.DIR)/boost-$(BOOST.VERSION)
BOOST.BUILD=$(DOWNLOADS.DIR)/build.boost-$(BOOST.VERSION)
boost: boost.fetch boost.build
boost.fetch: .FORCE
	rm -rf $(BOOST.DIR) && rm -f $(BOOST.ARCHIVE)
	cd $(DOWNLOADS.DIR) && wget $(BOOST.URL) -O $(BOOST.ARCHIVE) && tar xvf $(BOOST.ARCHIVE)

boost.build: .FORCE
	cd $(BOOST.DIR) && ./bootstrap.sh --prefix=$(INSTALLED.HOST.DIR) && ./b2 stage threading=multi link=shared --without-python && ./b2 install threading=multi link=shared --without-python

VTK.VERSION=9.4.0
VTK.DIR=$(DOWNLOADS.DIR)/VTK-$(VTK.VERSION)
VTK.BUILD=$(DOWNLOADS.DIR)/build.vtk
VTK.ARCHIVE=VTK-$(VTK.VERSION).tar.gz
VTK.URL=https://www.vtk.org/files/release/9.4/$(VTK.ARCHIVE)
vtk: vtk.fetch vtk.build
vtk.fetch:. .FORCE
	rm -rf $(DOWNLOADS.DIR)/$(VTK.ARCHIVE) && rm -rf $(VTK.DIR) && mkdir -p $(DOWNLOADS.DIR)
	cd $(DOWNLOADS.DIR) && wget $(VTK.URL) && tar xf $(VTK.ARCHIVE)

vtk.build: .FORCE
	mkdir -p $(VTK.BUILD)
	cd $(VTK.BUILD) && cmake -DCMAKE_INSTALL_PREFIX=$(INSTALLED.HOST.DIR) -DCMAKE_PREFIX_PATH=$(INSTALLED.HOST.DIR) -DBUILD_SHARED_LIBS=ON -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DVTK_LEGACY_SILENT=ON -DBOOST_ROOT=$(INSTALLED.HOST.DIR) -DBOOST_LIBRARY_DIR=$(INSTALLED.HOST.DIR)/lib $(VTK.DIR) && make -j8 install

# https://github.com/PointCloudLibrary/pcl/releases/download/pcl-1.14.1/source.tar.gz
PCL.VERSION=1.14.0
PCL.DIR=$(DOWNLOADS.DIR)/pcl-$(PCL.VERSION)
PCL.BUILD=$(DOWNLOADS.DIR)/build.pcl-$(PCL.VERSION)
PCL.ARCHIVE=pcl-$(PCL.VERSION).tar.gz
PCL.URL=https://github.com/PointCloudLibrary/pcl/releases/download/pcl-$(PCL.VERSION)/source.tar.gz
pcl: pcl.fetch pcl.build
pcl.fetch: .FORCE
	mkdir -p $(DOWNLOADS.DIR) && mkdir -p $(PCL.DIR)
	cd $(DOWNLOADS.DIR) && wget $(PCL.URL) -O $(PCL.ARCHIVE) && tar xf $(PCL.ARCHIVE) -C $(PCL.DIR)

pcl.build: .FORCE
	mkdir -p $(PCL.BUILD)
	cd $(PCL.BUILD) && cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=$(INSTALLED.HOST.DIR) -DCMAKE_INSTALL_PREFIX=$(INSTALLED.HOST.DIR) -DVTK_DIR=$(INSTALLED.HOST.DIR) -DBUILD_io=OFF $(PCL.DIR)/pcl && make -j8 install

.FORCE:

