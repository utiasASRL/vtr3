
NAME = pnmutils
LIB_EXT = a
LIBRARY	= lib$(NAME).$(LIB_EXT)

EXEC =

INCLUDE = \
	pnmutils.h \
	pgrpnmio.h


LIBSRC = \
	pnmutils.cpp \
        pgrpnmio.cpp


LIBOBJ = $(LIBSRC:%.cpp=%.o) 

ARFLAGS = rv
CPPFLAGS += -Wall -O3 -DNDEBUG
CPPFLAGS += -I/usr/include/triclops

all: $(LIBRARY)

%.o: %.cpp
		$(CXX) $(CXXFLAGS) $(CPPFLAGS) -c $< -o $@

$(LIBRARY): $(LIBOBJ)
		$(AR) $(ARFLAGS) $@ $^

clean:
		rm -f $(LIBOBJ) $(LIBRARY)

