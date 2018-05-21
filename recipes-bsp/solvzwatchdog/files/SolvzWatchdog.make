ifndef config
  config=gallant
endif

ifndef verbose
  SILENT = @
endif

ifdef cross
SYSROOT=../gallant/fsl-community-bsp/build/tmp/sysroots/gallant
TOOLSPATH=/opt/gcc-linaro-4.9-2015.05-x86_64_arm-linux-gnueabihf
CROSS_COMPILE=arm-linux-gnueabihf-

PATH:=$(PATH):$(TOOLSPATH)/bin
CXX=$(CROSS_COMPILE)g++
CFLAGS+=-I$(SYSROOT)/usr/include/
CFLAGS+=--sysroot=$(SYSROOT)
LDFLAGS+=-L$(SYSROOT)/usr/lib
LDFLAGS+=-L$(SYSROOT)/lib
LDFLAGS+=--sysroot=$(SYSROOT)
endif

.PHONY: clean prebuild prelink


ifeq ($(config),gallant)
  RESCOMP = windres
  TARGETDIR = .
  TARGET = $(TARGETDIR)/SolvzWatchdogGallant
  OBJDIR = obj/Gallant
  DEFINES += -DNDEBUG
  INCLUDES += -I.
  FORCE_INCLUDE +=
  ALL_CPPFLAGS += $(CPPFLAGS) -MMD -MP $(DEFINES) $(INCLUDES)
  ALL_CFLAGS += $(CFLAGS) $(ALL_CPPFLAGS) -Wall -Wextra -O3 -Wno-write-strings
  ALL_CXXFLAGS += $(CXXFLAGS) $(ALL_CFLAGS)
  ALL_RESFLAGS += $(RESFLAGS) $(DEFINES) $(INCLUDES)
  LIBS += -lpthread
  LDDEPS +=
  ALL_LDFLAGS += $(LDFLAGS) -Wl,-x 
  LINKCMD = $(CXX) -o $(TARGET) $(OBJECTS) $(ALL_LDFLAGS) $(LIBS)
  define PREBUILDCMDS
  endef
  define PRELINKCMDS
  endef
  define POSTBUILDCMDS
  endef
all: $(TARGETDIR) $(OBJDIR) prebuild prelink $(TARGET)
	@:

endif


ifeq ($(config),x86linux)
  RESCOMP = windres
  TARGETDIR = .
  TARGET = $(TARGETDIR)/SolvzWatchdog
  OBJDIR = obj/x86Linux
  DEFINES += -DNDEBUG
  INCLUDES += -I.
  FORCE_INCLUDE +=
  ALL_CPPFLAGS += $(CPPFLAGS) -MMD -MP $(DEFINES) $(INCLUDES)
  ALL_CFLAGS += $(CFLAGS) $(ALL_CPPFLAGS) -Wall -Wextra -O3 -Wno-write-strings
  ALL_CXXFLAGS += $(CXXFLAGS) $(ALL_CFLAGS)
  ALL_RESFLAGS += $(RESFLAGS) $(DEFINES) $(INCLUDES)
  LIBS += -lpthread
  LDDEPS +=
  ALL_LDFLAGS += $(LDFLAGS) -Wl,-x
  LINKCMD = $(CXX) -o $(TARGET) $(OBJECTS) $(ALL_LDFLAGS) $(LIBS)
  define PREBUILDCMDS
  endef
  define PRELINKCMDS
  endef
  define POSTBUILDCMDS
  endef
all: $(TARGETDIR) $(OBJDIR) prebuild prelink $(TARGET)
	 @:

endif

OBJECTS := \
	$(OBJDIR)/watchdogtest.o \


SHELLTYPE := msdos
ifeq (,$(ComSpec)$(COMSPEC))
  SHELLTYPE := posix
endif
ifeq (/bin,$(findstring /bin,$(SHELL)))
  SHELLTYPE := posix
endif

$(TARGET): $(GCH) $(OBJECTS) $(LDDEPS)
	@echo Linking SolvzWatchdog
	$(SILENT) $(LINKCMD)
	$(POSTBUILDCMDS)

$(TARGETDIR):
	@echo Creating $(TARGETDIR)
ifeq (posix,$(SHELLTYPE))
	$(SILENT) mkdir -p $(TARGETDIR)
else
	$(SILENT) mkdir $(subst /,\\,$(TARGETDIR))
endif

$(OBJDIR):
	@echo Creating $(OBJDIR)
ifeq (posix,$(SHELLTYPE))
	$(SILENT) mkdir -p $(OBJDIR)
else
	$(SILENT) mkdir $(subst /,\\,$(OBJDIR))
endif

clean:
	@echo Cleaning SolvzWatchdog
ifeq (posix,$(SHELLTYPE))
	$(SILENT) rm -f  $(TARGET)
	$(SILENT) rm -rf $(OBJDIR)
else
	$(SILENT) if exist $(subst /,\\,$(TARGET)) del $(subst /,\\,$(TARGET))
	$(SILENT) if exist $(subst /,\\,$(OBJDIR)) rmdir /s /q $(subst /,\\,$(OBJDIR))
endif



prebuild:
	$(PREBUILDCMDS)

prelink:
	$(PRELINKCMDS)

ifneq (,$(PCH))
$(OBJECTS): $(GCH) $(PCH)
$(GCH): $(PCH)
	@echo $(notdir $<)
	$(SILENT) $(CXX) -x c++-header $(ALL_CFLAGS) -o "$@" -MF "$(@:%.gch=%.d)" -c "$<"
endif

$(OBJDIR)/watchdogtest.o: watchdogtest.c
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(ALL_CFLAGS) $(FORCE_INCLUDE) -o "$@" -MF "$(@:%.o=%.d)" -c "$<"
$(OBJDIR)/%.o: src/%.c
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(ALL_CFLAGS) $(FORCE_INCLUDE) -o "$@" -MF "$(@:%.o=%.d)" -c "$<"

-include $(OBJECTS:%.o=%.d)
ifneq (,$(PCH))
  -include $(OBJDIR)/$(notdir $(PCH)).d
endif
