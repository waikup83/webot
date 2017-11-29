# Webots Makefile system 
#
# You may add some variable definitions hereafter to customize the build process
# See documentation in $(WEBOTS_HOME_PATH)/resources/Makefile.include


RESOURCES_PATH = $(WEBOTS_HOME)/resources/projects/robots/youbot
INCLUDE = -I"$(RESOURCES_PATH)/libraries/youbot_control/include"
LIBRARIES = -L"$(RESOURCES_PATH)/libraries/youbot_control" -lyoubot_control

YOUBOT_LIB = ../../lib/

### Do not modify: this includes Webots global Makefile.include
space :=
space +=
WEBOTS_HOME_PATH=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))
include $(WEBOTS_HOME_PATH)/resources/Makefile.include
