ifndef config
  config=gallant
endif

ifndef verbose
  SILENT = @
endif

ifeq ($(config),gallant)
  SolvzWatchdog_config = gallant
endif
ifeq ($(config),x86linux)
  SolvzWatchdog_config = x86linux
endif

PROJECTS := SolvzWatchdog

.PHONY: all clean help $(PROJECTS)

all: $(PROJECTS)

SolvzWatchdog:
ifneq (,$(SolvzWatchdog_config))
	@echo "==== Building SolvzWatchdog ($(SolvzWatchdog_config)) ===="
	@${MAKE} --no-print-directory -C . -f SolvzWatchdog.make config=$(SolvzWatchdog_config)
endif

clean:
	@${MAKE} --no-print-directory -C . -f SolvzWatchdog.make clean

help:
	@echo "Usage: make [config=name] [target]"
	@echo ""
	@echo "CONFIGURATIONS:"
	@echo "  gallant"
	@echo "  x86linux"
	@echo ""
	@echo "TARGETS:"
	@echo "   all (default)"
	@echo "   clean"
	 @echo "   SolvzWatchdog"
