# Webots Makefile system 
#
# You may add some variable definitions hereafter to customize the build process
# See documentation in $(WEBOTS_HOME_PATH)/resources/Makefile.include


YOUBOT_LIB = ../../lib/
C_SOURCES = \
  ReelYoubot.c \
  $(YOUBOT_LIB)base.c \
  $(YOUBOT_LIB)gripper.c \
  $(YOUBOT_LIB)arm.c \
  $(YOUBOT_LIB)tiny_math.c

### Do not modify: this includes Webots global Makefile.include
space :=
space +=
WEBOTS_HOME_PATH=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))
include $(WEBOTS_HOME_PATH)/resources/Makefile.include
