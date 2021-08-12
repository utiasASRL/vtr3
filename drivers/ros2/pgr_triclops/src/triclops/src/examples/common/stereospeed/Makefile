#####################################################################
# Copyright (c) 2001 Point Grey Research, Inc. All Rights Reserved.
#
# This software is the confidential and proprietary information of Point
# Grey Research, Inc. ("Confidential Information").  You shall not
# disclose such Confidential Information and shall use it only in
# accordance with the terms of the license agreement you entered into
# with PGR.
#
# PGR MAKES NO REPRESENTATIONS OR WARRANTIES ABOUT THE SUITABILITY OF THE
# SOFTWARE, EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
# IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
# PURPOSE, OR NON-INFRINGEMENT. PGR SHALL NOT BE LIABLE FOR ANY DAMAGES
# SUFFERED BY LICENSEE AS A RESULT OF USING, MODIFYING OR DISTRIBUTING
# THIS SOFTWARE OR ITS DERIVATIVES.
#####################################################################
#
#####################################################################
TOP	= ../../../..

ifdef BUILD_EXAMPLE
   include ../../Make/$(BUILD_EXAMPLE)
else # default build conf
   include ../../Make/default
endif

CPPFLAGS += -I.
CPPFLAGS += -DLINUX -UWIN32 -UWIN64
 
CXXFLAGS += -Wall -O3

LIBS    += -ltriclops -lpnmutils -lrt -ldl

EXEC1NAME = stereospeed
EXEC1SRC  = Timer.cpp $(EXEC1NAME).cpp 
EXEC1     = $(EXEC1NAME)$(DEBUG_SUFFIX)
EXECS     = $(EXEC1)

all:	bin

bin: $(EXECS)

$(EXEC1): $(EXEC1SRC:%.cpp=%.o)
	$(CXX) $(LDFLAGS) -o $@ $^ $(LIBS)

%.o:%.cpp
	$(CXX) -c $(CXXFLAGS) $(CPPFLAGS) $*.cpp

clean:
	rm -f *~ *.o *.d $(EXECS)
	
include ../../Make/common	
