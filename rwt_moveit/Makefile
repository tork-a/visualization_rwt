include $(shell rospack find mk)/cmake.mk
all: make
clean: myclean
VS060_DIR = `rospack find vs060`
ROS3DJS_URL = "https://github.com/KazutoMurase/ros3djs.git"
make:
	-mkdir -p ./www/lib
	-cd ./www/lib && git clone $(ROS3DJS_URL)
	-mkdir -p $(VS060_DIR)/www
	-ln -s -T $(VS060_DIR)/model $(VS060_DIR)/www/model

myclean:
	-rm -rf ./www/lib
	-rm -rf $(VS060_DIR)/www
