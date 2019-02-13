BUILD_CORES=32

all: 
		#scons -j $(BUILD_CORES) --ignore-style build/ARM/gem5.debug
		python `which scons` -j $(BUILD_CORES) --ignore-style build/ARM/gem5.opt
		#scons -j $(BUILD_CORES) --ignore-style build/ALPHA/gem5.debug
		#scons -j $(BUILD_CORES) --ignore-style build/ALPHA/gem5.opt

clean:
		rm -rf  build
