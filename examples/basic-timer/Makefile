CONTIKI_PROJECT = basic
all: $(CONTIKI_PROJECT)

DEFINES+=PROJECT_CONF_H=\"project-conf.h\"
CONTIKI = ../..
include $(CONTIKI)/Makefile.include

download:
	sh ./dslite.sh
	#sh ./cc2650_dslite.sh
